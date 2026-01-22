#!/usr/bin/env python3
import os, csv, cv2, rospy, rospkg
from threading import Lock
from collections import deque
from datetime import datetime

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Empty, Bool, Float64MultiArray

try:
    from scipy.spatial.transform import Rotation as R
    from scipy.optimize import curve_fit
except Exception as e:
    R = None
    curve_fit = None

try:
    from com_3d.com_estimation import tau_app_model, tau_model
except Exception as e:
    tau_app_model = None
    tau_model = None

# Optional helper (nice but not required)
try:
    from com_3d.helper_fns import enforce_quat_continuity
except Exception as e:
    enforce_quat_continuity = None


class StreamingCOM3DEstimator:
    """
    Streaming parameter estimator for (m, zc). theta_star is derived from zc and rc0_known.

    Design:
      - add_sample(...) is called at FT rate (or whatever clock you're using)
      - a rospy.Timer calls update_estimate() at a slower rate (e.g., 5 Hz)
      - update_estimate() runs a tiny non-linear least squares on a sliding window
    """
    def __init__(self):
        # Geometry/config
        self.o_obj = np.array(rospy.get_param('/com_3d/o_obj', [0.0, 0.0, 0.0]), dtype=float).reshape(3)
        self.rc0_known = np.array(rospy.get_param('/com_3d/rc0_known', [0.05, 0.0, 0.0]), dtype=float).reshape(3)
        self.tag_cam_to_world_euler_deg = rospy.get_param('~tag_cam_to_world_euler_deg', [0.0, 0.0, 0.0])

        # Estimator cadence/windowing
        self.window_size = int(rospy.get_param('~est_window_size', 120))
        self.min_samples = int(rospy.get_param('~est_min_samples', 30))
        self.use_contact_gate = bool(rospy.get_param('~est_use_contact_gate', True))
        self.use_trigger_gate = bool(rospy.get_param('~est_use_trigger_gate', True))

        # Simple streaming force filter
        self.medwin = int(rospy.get_param('~force_medwin', 5))
        self.lpf_beta = float(rospy.get_param('~force_lpf_beta', 0.25))  # 0..1, higher = less smoothing

        # State
        self._lock = Lock()
        self._f_hist = deque(maxlen=max(3, self.medwin))
        self._f_filt = None

        self._q0 = None  # reference quat for relative rotation
        self._last_q = None

        self._th = deque(maxlen=self.window_size)
        self._axis = deque(maxlen=self.window_size)
        self._tau_app = deque(maxlen=self.window_size)
        self._contact = deque(maxlen=self.window_size)
        self._trigger = deque(maxlen=self.window_size)

        self.last_est = None  # (m, zc, theta_star, tip_axis, n_used)

        if R is None:
            rospy.logwarn("[stream_est] scipy Rotation unavailable; theta will not be computed.")
        if tau_app_model is None or tau_model is None:
            rospy.logwarn("[stream_est] com_3d.com_estimation unavailable; estimation will not run.")
        if curve_fit is None:
            rospy.logwarn("[stream_est] scipy curve_fit unavailable; estimation will not run.")

    def reset(self):
        with self._lock:
            self._f_hist.clear()
            self._f_filt = None
            self._q0 = None
            self._last_q = None
            self._th.clear()
            self._axis.clear()
            self._tau_app.clear()
            self._contact.clear()
            self._trigger.clear()
            self.last_est = None

    def _sanitize_quat(self, qxyzw):
        q = np.asarray(qxyzw, dtype=float).reshape(4)
        if not np.isfinite(q).all():
            return None
        n = float(np.linalg.norm(q))
        if n < 1e-9:
            return None
        q = q / n
        # continuity: flip sign to stay close to previous
        if self._last_q is not None and float(np.dot(self._last_q, q)) < 0.0:
            q = -q
        self._last_q = q
        return q

    def _theta_and_axis(self, qxyzw):
        """
        Compute (theta, axis_hat) consistent with analyze_experiment:
          - relative rotation from first seen quaternion
          - convert to rotvec
          - rotate rotvec from camera frame into world using fixed R_cam_to_world
        """
        if R is None:
            return None, None
        q = self._sanitize_quat(qxyzw)
        if q is None:
            return None, None

        if self._q0 is None:
            self._q0 = q.copy()

        R0 = R.from_quat(self._q0)     # [x,y,z,w]
        Rq = R.from_quat(q)
        R_rel = R0.inv() * Rq
        rotvec_cam = R_rel.as_rotvec()

        eul = np.asarray(self.tag_cam_to_world_euler_deg, dtype=float)
        R_cam_to_world = R.from_euler('xyz', eul, degrees=True)
        rotvec_world = (R_cam_to_world * R.from_rotvec(rotvec_cam)).as_rotvec()

        th = float(np.linalg.norm(rotvec_world))
        if th < 1e-9:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            axis = rotvec_world / th
        return th, axis

    def _filter_force(self, f_xyz):
        f = np.asarray(f_xyz, dtype=float).reshape(3)
        if not np.isfinite(f).all():
            return None
        self._f_hist.append(f)
        f_med = np.median(np.stack(self._f_hist, axis=0), axis=0) if len(self._f_hist) else f
        if self._f_filt is None:
            self._f_filt = f_med
        else:
            self._f_filt = (1.0 - self.lpf_beta) * self._f_filt + self.lpf_beta * f_med
        return self._f_filt.copy()

    def add_sample(self, f_xyz, ee_xyz, tag_quat_xyzw, contact_flag, trigger_flag):
        """
        Add one sample. This is cheap (no optimization). Optimization happens in update_estimate().
        """
        with self._lock:
            th, axis = self._theta_and_axis(tag_quat_xyzw) if tag_quat_xyzw is not None else (None, None)
            if th is None or axis is None:
                return False

            f_filt = self._filter_force(f_xyz)
            if f_filt is None:
                return False

            ee_xyz = np.asarray(ee_xyz, dtype=float).reshape(3)
            if not np.isfinite(ee_xyz).all():
                return False

            rf = ee_xyz - self.o_obj
            f_app = -f_filt  # match analyze_experiment sign convention
            if tau_app_model is None:
                return False
            tau_app = tau_app_model(f_app.reshape(1, 3), rf.reshape(1, 3)).reshape(3)

            self._th.append(th)
            self._axis.append(axis)
            self._tau_app.append(tau_app)
            self._contact.append(int(contact_flag))
            self._trigger.append(int(trigger_flag))
            return True

    def update_estimate(self):
        """
        Run a small fit on the latest window and store last_est.
        Returns last_est or None.
        """
        if curve_fit is None or tau_model is None:
            return None

        with self._lock:
            if len(self._th) < self.min_samples:
                return None

            th = np.asarray(self._th, dtype=float)
            axis = np.asarray(self._axis, dtype=float)
            tau_app = np.asarray(self._tau_app, dtype=float)
            contact = np.asarray(self._contact, dtype=int)
            trigger = np.asarray(self._trigger, dtype=int)

        # Optional gating: only use data after contact and before trigger (as it streams)
        idx = np.arange(len(th))
        if self.use_contact_gate and np.any(contact == 1):
            c0 = int(np.where(contact == 1)[0][0])
            idx = idx[idx >= c0]
        if self.use_trigger_gate and np.any(trigger == 1):
            s0 = int(np.where(trigger == 1)[0][0])
            idx = idx[idx <= s0]
        if len(idx) < self.min_samples:
            return None

        th_use = th[idx]
        tau_use = tau_app[idx, :]
        tip_axis = axis[idx, :].mean(axis=0)
        tip_axis = tip_axis / (np.linalg.norm(tip_axis) + 1e-12)

        # Cheap initialization:
        # use previous estimate if available, else a generic starting point
        if self.last_est is not None:
            m0, zc0, *_ = self.last_est
            p0 = [max(float(m0), 1e-6), max(float(zc0), 1e-6)]
        else:
            p0 = [0.5, 0.10]  # reasonable-ish defaults

        rc0 = self.rc0_known.copy()

        # Make curve_fit happy: flatten y and h(theta,params)
        def model(th_vec, m, zc):
            y = tau_model(th_vec, m, zc, rc0_known=rc0, e_hat=tip_axis)
            return np.asarray(y, dtype=float).reshape(-1)

        ydata = np.asarray(tau_use, dtype=float).reshape(-1)

        try:
            popt, _ = curve_fit(
                model,
                th_use,
                ydata,
                p0=p0,
                bounds=([0.0, 0.0], [np.inf, np.inf]),
                maxfev=2000
            )
            m_est, zc_est = float(popt[0]), float(popt[1])
        except Exception as e:
            rospy.logdebug("[stream_est] curve_fit failed: %s", str(e))
            return None

        d_xy = float(np.linalg.norm(self.rc0_known[:2]))
        theta_star = float(np.arctan2(d_xy, zc_est)) if zc_est > 1e-12 else float('nan')

        est = (m_est, zc_est, theta_star, tip_axis, int(len(th_use)))
        self.last_est = est
        return est
    



class FTClockLogger:
    """
    Drop-in replacement for sync_logger.py that ALSO runs a streaming estimator.

    - Still logs CSV/video exactly like before
    - While recording, continuously updates (m, zc, theta_star) from the live stream
    - Publishes /com_3d/online_estimate continuously (latched) and prints final on stop
    """
    def __init__(self):
        self.run_base   = rospy.get_param('~run_base', None)
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.ee_frame   = rospy.get_param('~ee_frame',   'finger_tip')

        self.ft_stream_topic    = rospy.get_param('~ft_topic',   '/netft_data_transformed')
        self.tag_topic          = rospy.get_param('~dets_topic_name', '/tag_detections')
        self.image_topic        = rospy.get_param('~image_topic', '/tag_detections_image')

        self.flush_period   = float(rospy.get_param('~flush_period', 0.25))
        self.fps_hint       = float(rospy.get_param('~fps_hint', 30.0))

        self.tip_pose_max_age = float(rospy.get_param('~tip_pose_max_age', 0.02))
        self.tip_pose_buf_len = int(rospy.get_param('~tip_pose_buf_len', 400))
        self._tip_buf       = deque(maxlen=self.tip_pose_buf_len)

        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        pkg = rospkg.RosPack().get_path('com_3d')
        self.out_dir = os.path.join(pkg, 'experiments'); os.makedirs(self.out_dir, exist_ok=True)
        self.traj_idx = 0
        self.recording = False
        self.csv_f = self.csv_w = None
        self.csv_lock = Lock()
        self.last_flush = rospy.Time.now().to_sec()

        # --- video state ---
        self.bridge = CvBridge()
        self.writer = None
        self.video_path = None

        # FT state (contact, triggered, etc.)
        self.ft_contact_flag = 0
        self.ft_trigger_flag = 0

        # latest tag (sample-and-hold)
        self.tag_latest = None  # {'t':..., 'id':..., 'tq':(qx,qy,qz,qw), 'txyz':(...)}

        # EE pose fallback
        self._EE_lock = Lock()
        self.ee_last = [0.0] * 7

        # Streaming estimator
        self.enable_streaming_est = bool(rospy.get_param('~enable_streaming_est', True))
        self.est = StreamingCOM3DEstimator() if self.enable_streaming_est else None
        self.est_pub = rospy.Publisher('/com_3d/online_estimate', Float64MultiArray, queue_size=1, latch=True)

        est_rate_hz = float(rospy.get_param('~est_update_hz', 5.0))
        self._est_timer = rospy.Timer(rospy.Duration(1.0 / max(est_rate_hz, 1e-6)), self._on_est_timer)

        rospy.Subscriber(self.ft_stream_topic, WrenchStamped, self._on_ft, queue_size=500)
        rospy.Subscriber(self.tag_topic, AprilTagDetectionArray, self._on_tag, queue_size=50)
        rospy.Subscriber('/com_3d/log_start', Empty, self._start, queue_size=1)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._stop,  queue_size=1)
        rospy.Subscriber('/com_3d/fw_contact_status', Bool, self._on_fw_contact)
        rospy.Subscriber('/com_3d/fw_trigger_status', Bool, self._on_fw_trigger)
        rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=10, buff_size=2**24)
        rospy.Subscriber("/com_3d/tip_pose_fk", PoseStamped, self._on_tip_pose, queue_size=200)

    def _start(self, _):
        with self.csv_lock:
            if self.recording:
                return
            self.traj_idx += 1
            run = self.run_base or f"{self.timestamp}"
            fname = f"{run}_t{self.traj_idx:02d}.csv"
            path = os.path.join(self.out_dir, fname)

            self.csv_f = open(path, 'w', newline='')
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
            self.recording = True
            self.last_flush = rospy.Time.now().to_sec()

            # reset state
            self.writer = None
            self.ft_contact_flag = 0
            self.ft_trigger_flag = 0
            if self.est is not None:
                self.est.reset()

        rospy.loginfo("[ft_clock_logger] START -> %s", path)

    def _stop(self, _):
        with self.csv_lock:
            if not self.recording:
                return
            self.recording = False
            # stop video first
            try:
                if self.writer is not None:
                    self.writer.release()
            except Exception:
                pass
            self.writer = None
            # close CSV
            try:
                if self.csv_f:
                    self.csv_f.flush()
                    self.csv_f.close()
            finally:
                self.csv_f = None
                self.csv_w = None

        # Print final estimate (best we have so far)
        if self.est is not None and self.est.last_est is not None:
            m, zc, ths, tip_axis, n = self.est.last_est
            rospy.loginfo("[ft_clock_logger] FINAL STREAM EST -> m=%.4f kg, zc=%.4f m, theta*=%.2f deg (n=%d)",
                          m, zc, np.rad2deg(ths), n)
        rospy.loginfo("[ft_clock_logger] STOP")

    def _on_tag(self, msg: AprilTagDetectionArray):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        if not msg.detections:
            with self.csv_lock:
                self.tag_latest = None
            return

        det = msg.detections[0]
        p = det.pose.pose.pose
        q = p.orientation
        tx, ty, tz = p.position.x, p.position.y, p.position.z
        tag_id = det.id[0] if det.id else -1

        with self.csv_lock:
            self.tag_latest = {'t': t, 'id': int(tag_id),
                               'tq': (q.x, q.y, q.z, q.w),
                               'txyz': (tx, ty, tz)}

    def _on_image(self, msg: Image):
        # Keep your video behavior unchanged (sample-and-hold frames while recording)
        with self.csv_lock:
            if not self.recording:
                return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        # lazy init writer
        if self.writer is None:
            h, w = frame.shape[:2]
            run = self.run_base or f"{self.timestamp}"
            vname = f"{run}_t{self.traj_idx:02d}.mp4"
            self.video_path = os.path.join(self.out_dir, vname)
            try:
                self.writer = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*'mp4v'), self.fps_hint, (w, h), True)
                if not self.writer.isOpened():
                    rospy.logerr("[ft_clock_logger] Failed to open VideoWriter for %s", self.video_path)
                    self.writer = None
                    return
            except Exception:
                self.writer = None
                return

        try:
            self.writer.write(frame)
        except Exception:
            pass

    def _on_fw_contact(self, msg: Bool):
        with self.csv_lock:
            if self.recording:
                self.ft_contact_flag = 1 if msg.data else 0

    def _on_fw_trigger(self, msg: Bool):
        with self.csv_lock:
            if self.recording:
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

    def _on_ft(self, msg: WrenchStamped):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        ft = (float(msg.wrench.force.x),  float(msg.wrench.force.y),  float(msg.wrench.force.z),
              float(msg.wrench.torque.x), float(msg.wrench.torque.y), float(msg.wrench.torque.z))

        with self.csv_lock:
            tag = self.tag_latest
            recording = self.recording
            csv_w = self.csv_w
            csv_f = self.csv_f

        if not recording or csv_w is None:
            return

        ee = self._get_tip_pose_at(t)

        # CSV row
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

        with self.csv_lock:
            # recording might have stopped while we waited
            if not self.recording or self.csv_w is None:
                return

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
            if (now - self.last_flush) >= self.flush_period and self.csv_f is not None:
                self.csv_f.flush()
                self.last_flush = now

        # Streaming estimator ingest (uses the live stream, before log_stop)
        if self.est is not None and tag is not None:
            self.est.add_sample(
                f_xyz=np.array(ft[0:3]),
                ee_xyz=np.array(ee[0:3]),
                tag_quat_xyzw=np.array(tag['tq']),
                contact_flag=self.ft_contact_flag,
                trigger_flag=self.ft_trigger_flag
            )

    def _on_est_timer(self, _evt):
        if self.est is None:
            return
        est = self.est.update_estimate()
        if est is None:
            return
        m, zc, ths, tip_axis, n = est
        msg = Float64MultiArray(data=[m, zc, ths])
        self.est_pub.publish(msg)

    def shutdown(self):
        try:
            if self.writer is not None:
                self.writer.release()
        except Exception:
            pass
        try:
            if self.csv_f:
                self.csv_f.flush()
                self.csv_f.close()
        except Exception:
            pass


if __name__ == '__main__':
    rospy.init_node('ft_clock_logger')
    node = FTClockLogger()
    rospy.on_shutdown(getattr(node, 'shutdown', lambda: None))
    rospy.spin()