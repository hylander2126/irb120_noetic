#!/usr/bin/env python3
import os, csv, cv2, threading
import numpy as np
import matplotlib.pyplot as plt
import rospy, rospkg
from datetime import datetime
from collections import deque
from threading import Lock

from std_msgs.msg import Float64MultiArray, Empty, Bool
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge

from com_3d.helper_fns import quat_normalize, quat_conj, quat_mul, quat_to_rotvec

FPS_HINT_DEFAULT = 30


class CoMPlotAndLogNode:
    """
    Single node that:
      - live-plots (gap-compressed) between log_start/log_stop
      - continuously records CSV + MP4 once a session is started
      - rotates sessions on log_start when object_name or n_safety changes
    """

    # --------------------------- init ---------------------------
    def __init__(self):
        rospy.init_node("com_plot_and_log", anonymous=True)

        # ---------- params (logger-ish) ----------
        self.run_base   = rospy.get_param('~run_base', None)

        self.ft_stream_topic = rospy.get_param('~ft_topic', '/netft_data_transformed')
        self.tag_topic       = rospy.get_param('~dets_topic_name', '/tag_detections')
        self.image_topic     = rospy.get_param('~image_topic', '/tag_detections_image')

        self.flush_period = float(rospy.get_param('~flush_period', 0.25))
        self.fps_hint     = float(rospy.get_param('~fps_hint', FPS_HINT_DEFAULT))

        self.tip_pose_max_age = float(rospy.get_param('~tip_pose_max_age', 0.02))
        self.tip_pose_buf_len = int(rospy.get_param('~tip_pose_buf_len', 400))
        self._tip_buf = deque(maxlen=self.tip_pose_buf_len)  # (t, [x,y,z,qx,qy,qz,qw])
        self._EE_lock = Lock()
        self.ee_last = [0.0] * 7

        # output dir
        pkg = rospkg.RosPack().get_path('com_3d')
        self.out_dir = os.path.join(pkg, 'experiments')
        os.makedirs(self.out_dir, exist_ok=True)
        self.launch_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # ---------- shared locks ----------
        self.lock = threading.Lock()     # plot data + fit results
        self.io_lock = Lock()            # csv/video/tag_latest/session rotation

        # ---------- logger state ----------
        self.csv_f = None
        self.csv_w = None
        self.writer = None
        self.bridge = CvBridge()

        # # ----- camera warmup / preview -----
        # self.warmup_decode = bool(rospy.get_param('~warmup_decode', True)) # decode even when not recording
        # self.warmup_frames = 0

        self.session_open = False
        self.session_key = None     # (object_name, n_safety, run_base)
        self.stem = None
        self.csv_path = None
        self.video_path = None
        # self.session_idx = 0
        self.last_flush = rospy.Time.now().to_sec()

        # continuous recording flag (once started for a session, always record until next rotation)
        self.recording = False

        # FT state flags
        self.ft_contact_flag = 0
        self.ft_trigger_flag = 0

        # tag sample/hold for CSV
        self.tag_latest = None
        self.last_tag_q = None

        # ---------- plotter state (from your LiveFitPlotter) ----------
        self.q_ref = None
        self.pending_result = None
        self.save_pending = False

        # plot gating (gap-compressed)
        self.is_live_plot = False
        self.t_offset = 0.0
        self.t_seg_start_wall = None
        self.has_started_once = False

        # buffers
        self.live_t_ft = []
        self.live_f = []        # [fx, fy, fz]
        self.live_t_tag = []
        self.live_th = []

        # fit plot artists
        self.estimate_count = 0
        self.fit_artists = []
        self.clear_artists_pending = False

        # update rate
        self.update_hz = 10.0
        self.dt_wall = 1.0 / self.update_hz

        # ---------- matplotlib setup ----------
        plt.ion()

        self.fig_live, self.ax_live = plt.subplots(1, 1, figsize=(8, 4.5))
        self.fig_live.canvas.manager.set_window_title("CoM Estimator - Live")

        self.ln_fx, = self.ax_live.plot([], [], 'r-', label='Fx', alpha=0.6)
        self.ln_fy, = self.ax_live.plot([], [], 'g-', label='Fy', alpha=0.6)
        self.ln_fz, = self.ax_live.plot([], [], 'b-', label='Fz', alpha=0.6)
        self.ax_live.axhline(0.0, color='c', linewidth=1.0)
        self.ax_live.set_ylabel("Force (N)", fontsize=16)
        self.ax_live.set_xlabel("Time (s) (gap-compressed)", fontsize=16)
        # self.ax_live.set_title("Live Data Stream", fontsize=16)
        self.ax_live.tick_params(axis='both', labelsize=14)

        self.ax_live_twin = self.ax_live.twinx()
        self.ln_th, = self.ax_live_twin.plot([], [], 'k-', linewidth=2, label='Angle')
        self.ax_live_twin.set_ylabel("Angle (deg)", fontsize=16)
        self.ax_live_twin.tick_params(axis='y', labelsize=14)

        lines1, labels1 = self.ax_live.get_legend_handles_labels()
        lines2, labels2 = self.ax_live_twin.get_legend_handles_labels()
        self.ax_live.legend(lines1 + lines2, labels1 + labels2, loc="upper left", fontsize=10)
        self.ax_live.grid(True)

        self.fig_fit, self.ax_fit = plt.subplots(1, 1, figsize=(8, 4.5))
        self.fig_fit.canvas.manager.set_window_title("CoM Estimator - Fit Results")

        self.ax_fit.set_xlabel("Object Angle (deg)", fontsize=16)
        self.ax_fit.tick_params(axis='both', labelsize=14)
        self.ax_fit.set_ylabel("Torque (N-m)", fontsize=16)
        self.ax_fit.grid(True)

        self.ln_gt_th = self.ax_fit.axvline(0, color='g', linestyle='--', linewidth=2, label='Ground Truth')
        self.ax_fit.axhline(0.0, color='c', linewidth=1.0)

        # ---------- ROS subscribers ----------
        rospy.Subscriber('/com_3d/log_start', Empty, self._on_log_start, queue_size=1)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._on_log_stop,  queue_size=1)

        rospy.Subscriber(self.ft_stream_topic, WrenchStamped, self._on_ft, queue_size=500)
        rospy.Subscriber(self.tag_topic, AprilTagDetectionArray, self._on_tag, queue_size=50)
        rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=10, buff_size=2**24)

        rospy.Subscriber("/com_3d/tip_pose_fk", PoseStamped, self._on_tip_pose, queue_size=200)
        rospy.Subscriber('/com_3d/fw_contact_status', Bool, self._on_fw_contact, queue_size=10)
        rospy.Subscriber('/com_3d/fw_trigger_status', Bool, self._on_fw_trigger, queue_size=10)

        rospy.Subscriber("/com_3d/online_fit_debug", Float64MultiArray, self._on_fit_result, queue_size=10)

        rospy.Subscriber('/com_3d/run_reset', Empty, self._on_run_reset, queue_size=1)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("[com_plot_and_log] ready")

        # ---- ALWAYS open video (idle) ----
        # with self.io_lock:
        #     key = self._make_session_key() # initial session
        #     self._open_session_locked(key) # opens CSV + sets video_path; doesnt open VideoWriter yet
        #     self.recording = False      # idle until log_start
        #     rospy.loginfo("[com_plot_and_log] idle session armed...")

    # --------------------------- session rotation ---------------------------
    def _close_session_locked(self):
        # stop writing
        self.recording = False

        # close video
        try:
            if self.writer is not None:
                self.writer.release()
        except Exception:
            pass
        self.writer = None

        # close csv
        try:
            if self.csv_f:
                self.csv_f.flush()
                self.csv_f.close()
        except Exception:
            pass
        self.csv_f = None
        self.csv_w = None

        # clear session
        self.session_open = False
        self.session_key = None
        self.stem = None
        self.csv_path = None
        self.video_path = None

    def _open_session_locked(self, key):
        obj, ns = key
        self.stem = f"{self.launch_stamp}_{obj}_{ns:.3f}" #_e{self.session_idx:02d}"

        # publish for downstream consumers (estimator/plot saves)
        rospy.set_param('/com_3d/current_log_stem', self.stem)
        rospy.set_param('/com_3d/current_log_dir',  self.out_dir)

        self.csv_path = os.path.join(self.out_dir, self.stem + ".csv")
        self.video_path = os.path.join(self.out_dir, self.stem + ".mp4")

        self.csv_f = open(self.csv_path, 'w', newline='')
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow([
            'ros_time_sec',
            'fx','fy','fz','mx','my','mz',
            'ee_x','ee_y','ee_z','ee_qx','ee_qy','ee_qz','ee_qw',
            'tag_visible','tag_id',
            'tag_qx','tag_qy','tag_qz','tag_qw',
            'tag_x','tag_y','tag_z',
            'fw_contact','ft_trigger'
        ])

        self.writer = None  # lazy-open on first frame
        self.last_flush = rospy.Time.now().to_sec()

        self.session_open = True
        self.session_key = key
        self.recording = True

        rospy.loginfo("[com_plot_and_log] session open: %s", self.csv_path)

    # --------------------------- log_start/log_stop ---------------------------
    def _on_log_start(self, _):
        # 1) Rotate or open session for recording
        with self.io_lock:
            obj = rospy.get_param('/com_3d/object_name', 'unknown')
            ns  = float(rospy.get_param('/com_3d/n_safety', 0.0))
            new_key = (str(obj), round(ns, 3))

            # If no session open or key changed, start new file
            if (not self.session_open) or (new_key != self.session_key):
                if self.session_open:
                    self._close_session_locked()
                self._open_session_locked(new_key)
            else:
                # same session, ensure we're recording
                self.recording = True

        # 2) Plotter "segment start": gap-compressed time resumes
        with self.lock:
            now = rospy.get_time()

            if not self.has_started_once:
                self.live_t_ft, self.live_f = [], []
                self.live_t_tag, self.live_th = [], []
                self.t_offset = 0.0
                self.estimate_count = 0
                self.clear_artists_pending = True
                self.has_started_once = True

                # reset reference orientation
                if self.last_tag_q is not None:
                    self.q_ref = self.last_tag_q
                else:
                    self.q_ref = None

                rospy.loginfo("[com_plot_and_log] plot first start: cleared buffers")
            else:
                rospy.loginfo("[com_plot_and_log] plot segment restart: resuming buffers")

            self.t_seg_start_wall = now
            self.is_live_plot = True
            self.pending_result = None
            self.save_pending = False

    def _on_log_stop(self, _):
        # IMPORTANT: do NOT stop recording (video/CSV must be continuous).
        # Only affects plot timing (freeze gap-compressed x-axis).
        with self.lock:
            if self.is_live_plot and self.t_seg_start_wall is not None:
                self.t_offset += rospy.get_time() - self.t_seg_start_wall
            self.is_live_plot = False
            self.t_seg_start_wall = None
        
        self._save_live_plot_now() # Save live plot regardless of estimator result
        rospy.loginfo("[com_plot_and_log] log_stop: plot frozen; recording continues")

    def _on_run_reset(self, _):
        with self.io_lock:
            if self.session_open:
                self._close_session_locked()
        self._save_live_plot_now() # Save live plot regardless of estimator result
        rospy.loginfo("[com_plot_and_log] run_reset: closed files; next log_start will open new run")

        # OPTIONAL: also reset the plotter state so each run is clean
        with self.lock:
            self.live_t_ft, self.live_f = [], []
            self.live_t_tag, self.live_th = [], []
            self.t_offset = 0.0
            self.t_seg_start_wall = None
            self.is_live_plot = False
            self.estimate_count = 0
            self.clear_artists_pending = True
            self.has_started_once = False
            self.q_ref = None
            self.pending_result = None
            self.save_pending = False

    def _save_live_plot_now(self):
        with self.io_lock:
            out_dir = self.out_dir
            stem = self.stem
        if stem is None:
            return

        live_png_path = os.path.join(out_dir, stem + "_live_plot.png")
        try:
            # draw then save
            self.fig_live.canvas.draw_idle()
            self.fig_live.canvas.flush_events()
            self.fig_live.savefig(live_png_path, dpi=300, bbox_inches="tight")
            rospy.loginfo("[com_plot_and_log] saved live plot: %s", live_png_path)
        except Exception as e:
            rospy.logwarn("[com_plot_and_log] live plot save failed: %s", str(e))


    # --------------------------- plot timing ---------------------------
    def _now_plot_time(self):
        now = rospy.get_time()
        if self.t_seg_start_wall is None:
            return self.t_offset
        return self.t_offset + (now - self.t_seg_start_wall)

    # --------------------------- callbacks ---------------------------
    def _on_fw_contact(self, msg: Bool):
        with self.io_lock:
            self.ft_contact_flag = 1 if msg.data else 0

    def _on_fw_trigger(self, msg: Bool):
        with self.io_lock:
            self.ft_trigger_flag = 1 if msg.data else 0

    def _on_tip_pose(self, msg: PoseStamped):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        p = msg.pose.position
        q = msg.pose.orientation
        ee = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        with self._EE_lock:
            self._tip_buf.append((t, ee))

    def _get_tip_pose_at(self, t_query: float):
        with self._EE_lock:
            buf = list(self._tip_buf)
        if not buf:
            return self.ee_last[:]
        t_best, ee_best = min(buf, key=lambda x: abs(x[0] - t_query))
        if abs(t_best - t_query) > self.tip_pose_max_age:
            return self.ee_last[:]
        self.ee_last = ee_best[:]
        return ee_best

    def _on_tag(self, msg: AprilTagDetectionArray):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

        # update sample/hold for logger
        if not msg.detections:
            with self.io_lock:
                self.tag_latest = None
            return

        det = msg.detections[0]
        p = det.pose.pose.pose
        q = p.orientation
        q_arr = [q.x, q.y, q.z, q.w]
        self.last_tag_q = q_arr

        tx, ty, tz = p.position.x, p.position.y, p.position.z
        tag_id = det.id[0] if det.id else -1

        with self.io_lock:
            self.tag_latest = {'t': t, 'id': int(tag_id),
                               'tq': (q.x, q.y, q.z, q.w),
                               'txyz': (tx, ty, tz)}

        # also feed plot (angle)
        with self.lock:
            if not self.is_live_plot:
                return
            if self.q_ref is None:
                self.q_ref = q_arr

            q_norm = quat_normalize(q_arr)
            q_ref_inv = quat_conj(self.q_ref)
            q_rel = quat_mul(q_ref_inv, q_norm)
            rv = quat_to_rotvec(q_rel, normalize=False)
            angle_rad = np.linalg.norm(rv)

            tp = self._now_plot_time()
            self.live_t_tag.append(tp)
            self.live_th.append(np.rad2deg(angle_rad))

    def _on_image(self, msg: Image):
        """
        Convert outside lock and get video frame
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        
        with self.io_lock:
            # If for some reason session is not open or not recording, skip
            if not self.session_open: # or not self.recording:
                return

            if self.writer is None:
                h, w = frame.shape[:2]
                self.writer = cv2.VideoWriter(
                    self.video_path,
                    cv2.VideoWriter_fourcc(*'mp4v'),
                    self.fps_hint,
                    (w, h),
                    True
                )
                if not self.writer.isOpened():
                    rospy.logerr("[com_plot_and_log] failed to open VideoWriter for %s", self.video_path)
                    self.writer = None
                    return
            
            # Only write frames when recording enabled
            if not self.recording:
                return

            try:
                self.writer.write(frame)
            except Exception:
                pass

    def _on_ft(self, msg: WrenchStamped):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        ft = (float(msg.wrench.force.x),  float(msg.wrench.force.y),  float(msg.wrench.force.z),
              float(msg.wrench.torque.x), float(msg.wrench.torque.y), float(msg.wrench.torque.z))

        # plot (forces)
        with self.lock:
            if self.is_live_plot:
                tp = self._now_plot_time()
                self.live_t_ft.append(tp)
                self.live_f.append([ft[0], ft[1], ft[2]])

        # csv (continuous while recording)
        with self.io_lock:
            if not self.session_open or not self.recording or self.csv_w is None:
                return

            tag = self.tag_latest
            ee  = self._get_tip_pose_at(t)

            if tag is not None:
                tq = tag['tq']
                txyz = tag['txyz']
                tag_row = (
                    1, tag['id'],
                    f"{tq[0]:.5f}", f"{tq[1]:.5f}", f"{tq[2]:.5f}", f"{tq[3]:.5f}",
                    f"{txyz[0]:.5f}", f"{txyz[1]:.5f}", f"{txyz[2]:.5f}"
                )
            else:
                tag_row = (0, -1, "nan", "nan", "nan", "nan", "nan", "nan", "nan")

            self.csv_w.writerow([
                f"{t:.6f}",
                f"{ft[0]:.5f}",  f"{ft[1]:.5f}",  f"{ft[2]:.5f}",
                f"{ft[3]:.5f}",  f"{ft[4]:.5f}",  f"{ft[5]:.5f}",
                f"{ee[0]:.5f}",  f"{ee[1]:.5f}",  f"{ee[2]:.5f}",
                f"{ee[3]:.5f}",  f"{ee[4]:.5f}",  f"{ee[5]:.5f}", f"{ee[6]:.5f}",
                *tag_row,
                self.ft_contact_flag, self.ft_trigger_flag
            ])

            now = rospy.Time.now().to_sec()
            if (now - self.last_flush) >= self.flush_period:
                try:
                    self.csv_f.flush()
                except Exception:
                    pass
                self.last_flush = now

    # --------------------------- fit result (plotter) ---------------------------
    def _on_fit_result(self, msg):
        rospy.loginfo("[com_plot_and_log] received fit result")

        d = np.array(msg.data, dtype=float)
        if d.size < 5:
            return

        try:
            idx = 0
            Np = int(d[idx]); idx += 1
            Nr = int(d[idx]); idx += 1

            th_p = d[idx:idx+Np]; idx += Np
            tau_p = d[idx:idx+Np]; idx += Np
            fit_p = d[idx:idx+Np]; idx += Np

            th_r = d[idx:idx+Nr]; idx += Nr
            tau_r = d[idx:idx+Nr]; idx += Nr
            fit_r = d[idx:idx+Nr]; idx += Nr

            th_star_est = float(d[idx]); idx += 1
            m_est = float(d[idx]); idx += 1
            z_est = float(d[idx]); idx += 1
        except Exception as e:
            rospy.logerr("[com_plot_and_log] fit payload parse failed: %s", str(e))
            return

        payload = {
            "th_p": np.rad2deg(th_p),
            "tau_p": tau_p,
            "fit_p": fit_p,
            "th_r": np.rad2deg(th_r),
            "tau_r": tau_r,
            "fit_r": fit_r,
            "th_star": np.rad2deg(th_star_est),
            "m": m_est,
            "z": z_est,
        }
        with self.lock:
            self.pending_result = payload
            self.save_pending = True

    def _clear_fit_artists(self):
        for artist in self.fit_artists:
            try:
                artist.remove()
            except Exception:
                pass
        self.fit_artists.clear()
        self.ax_fit.relim()
        self.ax_fit.autoscale_view()

    def _update_result_plot_and_save(self):
        with self.lock:
            if self.pending_result is None:
                return
            data = self.pending_result
            self.pending_result = None
            do_save = self.save_pending
            self.save_pending = False
            self.estimate_count += 1
            est_num = self.estimate_count

        gt_th_star_rad = rospy.get_param("/com_3d/theta_star", 0.0)
        gt_th_star_deg = np.rad2deg(gt_th_star_rad)

        n_safety = rospy.get_param("/com_3d/n_safety", 0.00)
        self.ax_live.set_title(f"Live Data Stream (n_safety={n_safety})")

        cc = ['tab:blue', 'tab:orange']
        c_idx = (est_num - 1) % len(cc)
        colors = cc[c_idx]
        labels = ["Push Fit", "Retract Fit"] if c_idx == 0 else ['_', '_']
        markers = ['o', 'd']
        alpha_push = [0.70, 0.95]
        alpha_retr = [0.70, 0.70]
        lp = ['-', '--'][c_idx]
        lr = lp

        scatter_push = self.ax_fit.scatter(data["th_p"], data["tau_p"],
            c=colors, alpha=alpha_push[0], s=30, label="_", marker=markers[0])

        fit_push, = self.ax_fit.plot(data["th_p"], data["fit_p"],
            color='black', linestyle=lp, linewidth=2, alpha=alpha_push[1],
            label=labels[0])

        scatter_retr = self.ax_fit.scatter(data["th_r"], data["tau_r"],
            c=colors, alpha=alpha_retr[0], s=30, label="_", marker=markers[1])

        fit_retr, = self.ax_fit.plot(data["th_r"], data["fit_r"],
            color='black', linestyle=lr, linewidth=2, alpha=alpha_retr[1],
            label=labels[1])

        vl = self.ax_fit.axvline(data["th_star"],
            color=colors, linestyle='-', linewidth=2, alpha=0.9,
            label=f'Est {est_num} θ*={data["th_star"]:.1f}°, m={data["m"]:.2f}kg, z={data["z"]:.3f}m')

        self.fit_artists.extend([scatter_push, fit_push, scatter_retr, fit_retr, vl])

        self.ln_gt_th.set_xdata([gt_th_star_deg, gt_th_star_deg])
        self.ln_gt_th.set_label(f'GT $\\theta^*$ ({gt_th_star_deg:.1f}$^\\circ$)')

        self.ax_fit.legend(loc="upper right", fontsize=10)
        self.ax_fit.relim()
        self.ax_fit.autoscale_view()

        # Save fit + live plot using the CURRENT stem/dir (session stem)
        if do_save:
            with self.io_lock:
                out_dir = self.out_dir
                stem = self.stem

            if stem is not None:
                fit_png_path  = os.path.join(out_dir, stem + "_fit_plot.png")
                live_png_path = os.path.join(out_dir, stem + "_live_plot.png")
                try:
                    self.fig_fit.canvas.draw_idle()
                    self.fig_fit.canvas.flush_events()
                    self.fig_fit.savefig(fit_png_path, dpi=300, bbox_inches="tight")
                    rospy.loginfo("[com_plot_and_log] saved: %s", fit_png_path)

                    self.fig_live.canvas.draw_idle()
                    self.fig_live.canvas.flush_events()
                    self.fig_live.savefig(live_png_path, dpi=300, bbox_inches="tight")
                    rospy.loginfo("[com_plot_and_log] saved: %s", live_png_path)
                except Exception as e:
                    rospy.logwarn("[com_plot_and_log] save failed: %s", str(e))

    # --------------------------- run loop ---------------------------
    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                should_clear = self.clear_artists_pending
                self.clear_artists_pending = False
            if should_clear:
                self._clear_fit_artists()

            with self.lock:
                t_ft = list(self.live_t_ft)
                f_data = np.array(self.live_f) if self.live_f else np.empty((0, 3))
                t_tag = list(self.live_t_tag)
                th_data = list(self.live_th)

            if len(t_ft) > 1 and len(f_data) == len(t_ft):
                self.ln_fx.set_data(t_ft, f_data[:, 0])
                self.ln_fy.set_data(t_ft, f_data[:, 1])
                self.ln_fz.set_data(t_ft, f_data[:, 2])
                self.ax_live.relim()
                self.ax_live.autoscale_view()

            if len(t_tag) > 1 and len(th_data) == len(t_tag):
                self.ln_th.set_data(t_tag, th_data)
                self.ax_live_twin.relim()
                self.ax_live_twin.autoscale_view()

            self._update_result_plot_and_save()

            self.fig_live.canvas.draw_idle()
            self.fig_fit.canvas.draw_idle()
            self.fig_live.canvas.flush_events()
            self.fig_fit.canvas.flush_events()
            plt.pause(self.dt_wall)

    # --------------------------- shutdown ---------------------------
    def shutdown(self):
        rospy.loginfo("[com_plot_and_log] shutting down...")
        with self.io_lock:
            try:
                if self.writer is not None:
                    self.writer.release()
            except Exception:
                pass
            self.writer = None

            try:
                if self.csv_f:
                    self.csv_f.flush()
                    self.csv_f.close()
            except Exception:
                pass
            self.csv_f = None
            self.csv_w = None

            self.session_open = False
            self.recording = False
        rospy.loginfo("[com_plot_and_log] closed csv/video")


if __name__ == "__main__":
    node = CoMPlotAndLogNode()
    node.run()
