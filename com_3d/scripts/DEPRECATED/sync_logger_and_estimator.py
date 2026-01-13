#!/usr/bin/env python3
"""
sync_logger_streaming_v2.py

True streaming 3D CoM estimator:
- Ingest FT/EE/tag at 500 Hz
- Apply *causal* filtering (median + Butterworth low-pass) to BOTH force and tag rotvec
- Decimate for estimation (e.g., 500 Hz -> 100 Hz)
- Update nonlinear fit on a sliding window at ~5-10 Hz
- Publish live estimate and (optionally) debug samples for live plotting

Topics:
  Subscribes:
    - /ft_sensor/netft_data (WrenchStamped) [default param ~ft_stream_topic]
    - /com_3d/fw_contact (Bool)  (from force_watcher)
    - /com_3d/fw_trigger (Bool)
    - /tag_detections (AprilTagDetectionArray) [default param ~tag_topic]
    - /camera/color/image_raw (Image) [optional]
    - /com_3d/tip_pose_fk (PoseStamped) [optional: if you already compute EE pose elsewhere]
  Publishes:
    - /com_3d/online_estimate (Float64MultiArray): [m, zc, theta_star]  (theta_star in rad)
    - /com_3d/online_fit_debug (Float64MultiArray): packed arrays for plotting (see below)

Debug message packing (/com_3d/online_fit_debug):
  data = [N,
          theta_deg[0..N-1],
          tau_y[0..N-1],
          tau_fit_y[0..N-1],
          theta_star_deg,
          m_est, zc_est]
"""
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

# Optional SciPy
try:
    from scipy.spatial.transform import Rotation as R
    from scipy.optimize import curve_fit
    from scipy.signal import butter, sosfilt, sosfilt_zi
except Exception:
    R = None
    curve_fit = None
    butter = None
    sosfilt = None
    sosfilt_zi = None

# Your model functions (must exist in your ROS env)
try:
    from com_3d.com_estimation import tau_app_model, tau_model
except Exception:
    tau_app_model = None
    tau_model = None


class CausalLPF:
    """
    Causal median + Butterworth SOS low-pass filter with persistent state.
    Designed for streaming at fixed fs.
    """
    def __init__(self, fs_hz: float, cutoff_hz: float, order: int, dim: int, medwin: int):
        self.dim = int(dim)
        self.medwin = max(1, int(medwin))
        self._hist = deque(maxlen=max(1, self.medwin))
        self._use_scipy = (butter is not None) and (sosfilt is not None) and (sosfilt_zi is not None)

        self.sos = None
        self.zi = None
        if self._use_scipy and cutoff_hz is not None and cutoff_hz > 0:
            self.sos = butter(int(order), float(cutoff_hz), fs=float(fs_hz), btype='low', output='sos')
            # zi has shape (n_sections, 2) for scalar; expand for vector dims
            zi0 = sosfilt_zi(self.sos)  # (n_sections, 2)
            self.zi = np.repeat(zi0[:, :, None], self.dim, axis=2)  # (n_sections, 2, dim)

    def reset(self):
        self._hist.clear()
        self.zi = None  # will be re-init lazily; ok

    def step(self, x):
        x = np.asarray(x, dtype=float).reshape(self.dim)
        if not np.isfinite(x).all():
            return None

        self._hist.append(x)
        x_med = np.median(np.stack(self._hist, axis=0), axis=0) if len(self._hist) else x

        if self.sos is None:
            return x_med.copy()

        # lazy init zi (we lost it on reset)
        if self.zi is None:
            zi0 = sosfilt_zi(self.sos)
            self.zi = np.repeat(zi0[:, :, None], self.dim, axis=2)

        # sosfilt supports axis=-1; we want vector dim => treat as samples length=1 with shape (1, dim)
        y, self.zi = sosfilt(self.sos, x_med.reshape(1, self.dim), zi=self.zi)
        return y.reshape(self.dim)


class StreamingCOM3DEstimator:
    """
    True streaming estimator. Call add_sample() at FT rate; call update_estimate() at ~5-10 Hz.
    """
    def __init__(self):
        # Geometry/config
        self.o_obj = np.array(rospy.get_param('/com_3d/o_obj', [0.0, 0.0, 0.0]), dtype=float).reshape(3)
        self.rc0_known = np.array(rospy.get_param('/com_3d/rc0_known', [0.05, 0.0, 0.0]), dtype=float).reshape(3)
        self.tag_cam_to_world_euler_deg = rospy.get_param('~tag_cam_to_world_euler_deg', [0.0, 0.0, 0.0])

        # Sampling + windowing
        self.fs_hz = float(rospy.get_param('~fs_hz', 500.0))
        self.decimate = int(rospy.get_param('~est_decimate', 5))         # 500Hz -> 100Hz
        self.window_size = int(rospy.get_param('~est_window_size', 250)) # at 100Hz => 2.5s window
        self.min_samples = int(rospy.get_param('~est_min_samples', 80))
        self.use_contact_gate = bool(rospy.get_param('~est_use_contact_gate', True))
        self.use_trigger_gate = bool(rospy.get_param('~est_use_trigger_gate', True))

        # Filtering (match offline spirit: aggressive 5 Hz low-pass)
        self.force_medwin = int(rospy.get_param('~force_medwin', 5))
        self.force_cutoff_hz = float(rospy.get_param('~force_cutoff_hz', 5.0))
        self.force_order = int(rospy.get_param('~force_order', 4))

        self.rotvec_medwin = int(rospy.get_param('~rotvec_medwin', 5))
        self.rotvec_cutoff_hz = float(rospy.get_param('~rotvec_cutoff_hz', 5.0))
        self.rotvec_order = int(rospy.get_param('~rotvec_order', 4))

        # Filters (causal)
        self.force_filt = CausalLPF(self.fs_hz, self.force_cutoff_hz, self.force_order, dim=3, medwin=self.force_medwin)
        self.rotvec_filt = CausalLPF(self.fs_hz, self.rotvec_cutoff_hz, self.rotvec_order, dim=3, medwin=self.rotvec_medwin)

        # State
        self._lock = Lock()
        self._q0 = None
        self._last_q = None
        self._sample_i = 0

        self._th = deque(maxlen=self.window_size)
        self._axis = deque(maxlen=self.window_size)
        self._tau_app = deque(maxlen=self.window_size)
        self._contact = deque(maxlen=self.window_size)
        self._trigger = deque(maxlen=self.window_size)

        self.last_est = None  # (m, zc, theta_star, tip_axis, n_used)
        self.last_debug = None  # dict with arrays for plotting

        # Fixed camera->world rotation for tag rotvec (if you need it)
        if R is not None:
            rpy = np.deg2rad(np.array(self.tag_cam_to_world_euler_deg, dtype=float).reshape(3))
            self.R_cam_to_world = R.from_euler('xyz', rpy).as_matrix()
        else:
            self.R_cam_to_world = np.eye(3)

    def reset(self):
        with self._lock:
            self._q0 = None
            self._last_q = None
            self._sample_i = 0
            self._th.clear()
            self._axis.clear()
            self._tau_app.clear()
            self._contact.clear()
            self._trigger.clear()
            self.last_est = None
            self.last_debug = None
        self.force_filt.reset()
        self.rotvec_filt.reset()

    def _sanitize_quat(self, qxyzw):
        q = np.asarray(qxyzw, dtype=float).reshape(4)
        if not np.isfinite(q).all():
            return None
        n = float(np.linalg.norm(q))
        if n < 1e-9:
            return None
        q = q / n
        # sign continuity
        if self._last_q is not None and float(np.dot(self._last_q, q)) < 0.0:
            q = -q
        self._last_q = q
        return q

    def _rotvec_world_from_quat(self, qxyzw):
        """
        Relative rotvec from first quaternion; then rotate into world; zero-out Z (tipping is planar).
        Returns rotvec_world (3,)
        """
        if R is None:
            return None
        q = self._sanitize_quat(qxyzw)
        if q is None:
            return None
        if self._q0 is None:
            self._q0 = q.copy()

        r0 = R.from_quat(self._q0)
        r  = R.from_quat(q)
        r_rel = r0.inv() * r
        rv = r_rel.as_rotvec()  # camera frame
        rvw = self.R_cam_to_world.dot(rv.reshape(3))
        rvw[2] = 0.0
        return rvw

    def add_sample(self, f_xyz, ee_xyz, tag_quat_xyzw, contact_flag: int, trigger_flag: int):
        """
        Called at FT rate. Filters + decimates into the estimation buffer.
        """
        if tau_app_model is None or tau_model is None:
            return False

        rvw = self._rotvec_world_from_quat(tag_quat_xyzw)
        if rvw is None:
            return False

        # Filter force & rotvec causally
        f_f = self.force_filt.step(f_xyz)
        rv_f = self.rotvec_filt.step(rvw)
        if f_f is None or rv_f is None:
            return False

        # Decimate for estimation (still log full-rate CSV elsewhere)
        self._sample_i += 1
        if self.decimate > 1 and (self._sample_i % self.decimate) != 0:
            return True

        th = float(np.linalg.norm(rv_f))
        if not np.isfinite(th):
            return False

        axis = np.zeros(3, dtype=float)
        if th > 1e-9:
            axis = rv_f / th

        ee = np.asarray(ee_xyz, dtype=float).reshape(3)
        rf = ee - self.o_obj

        # Use your existing model: returns 3-vector torque (or consistent with your offline)
        tau_app = tau_app_model(f_f.reshape(1, 3), rf.reshape(1, 3)).reshape(3)

        with self._lock:
            self._th.append(th)
            self._axis.append(axis)
            self._tau_app.append(tau_app)
            self._contact.append(int(contact_flag))
            self._trigger.append(int(trigger_flag))
        return True

    def update_estimate(self):
        """
        Fit (m, zc) on current window. Also prepares debug arrays for live plotting.
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

        # Init from last
        if self.last_est is not None:
            m0, zc0, *_ = self.last_est
            p0 = [max(float(m0), 1e-6), max(float(zc0), 1e-6)]
        else:
            p0 = [0.6, 0.12]

        rc0 = self.rc0_known.copy()

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
                maxfev=2500
            )
            m_est, zc_est = float(popt[0]), float(popt[1])
        except Exception as e:
            rospy.logdebug("[stream_est] curve_fit failed: %s", str(e))
            return None

        d_xy = float(np.linalg.norm(self.rc0_known[:2]))
        theta_star = float(np.arctan2(d_xy, zc_est)) if zc_est > 1e-12 else float('nan')

        # Debug arrays (Y component like your plot)
        try:
            tau_fit = tau_model(th_use, m_est, zc_est, rc0_known=rc0, e_hat=tip_axis).reshape(-1, 3)
            tau_y = tau_use[:, 1].reshape(-1)
            tau_fit_y = tau_fit[:, 1].reshape(-1)
            th_deg = np.rad2deg(th_use).reshape(-1)
            dbg = {
                "th_deg": th_deg,
                "tau_y": tau_y,
                "tau_fit_y": tau_fit_y,
                "theta_star_deg": float(np.rad2deg(theta_star)),
                "m": m_est,
                "zc": zc_est,
            }
        except Exception:
            dbg = None

        est = (m_est, zc_est, theta_star, tip_axis, int(len(th_use)))
        self.last_est = est
        self.last_debug = dbg
        return est


class FTClockLogger:
    def __init__(self):
        rospy.init_node("ft_clock_logger_streaming_v2")

        # Topics
        self.ft_stream_topic = rospy.get_param("~ft_stream_topic", "/ft_sensor/netft_data")
        self.tag_topic = rospy.get_param("~tag_topic", "/tag_detections")
        self.image_topic = rospy.get_param("~image_topic", "")
        self.tip_pose_topic = rospy.get_param("~tip_pose_topic", "/com_3d/tip_pose_fk")

        # Logging
        self.save_video = bool(rospy.get_param("~save_video", False))
        self.flush_period = float(rospy.get_param("~flush_period", 0.5))
        self.output_dir = rospy.get_param("~output_dir", "")
        self.base_name = rospy.get_param("~base_name", "")

        # Estimator
        self.est = None #StreamingCOM3DEstimator()
        self.est_update_hz = float(rospy.get_param("~est_update_hz", 5.0))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))

        self.est_pub = rospy.Publisher("/com_3d/online_estimate", Float64MultiArray, queue_size=5)
        self.dbg_pub = rospy.Publisher("/com_3d/online_fit_debug", Float64MultiArray, queue_size=2)

        self.bridge = CvBridge()
        self.csv_lock = Lock()

        self.recording = False
        self.csv_f = None
        self.csv_w = None
        self.last_flush = 0.0

        self.tag_latest = None  # dict with tq (quat xyzw) and tx/ty/tz if needed
        self.last_image = None
        self.tip_pose_latest = None

        self.ft_contact_flag = 0
        self.ft_trigger_flag = 0

        # Subscribers
        rospy.Subscriber(self.ft_stream_topic, WrenchStamped, self._on_ft, queue_size=200)
        rospy.Subscriber("/com_3d/fw_contact", Bool, self._on_fw_contact, queue_size=50)
        rospy.Subscriber("/com_3d/fw_trigger", Bool, self._on_fw_trigger, queue_size=50)
        rospy.Subscriber(self.tag_topic, AprilTagDetectionArray, self._on_tag, queue_size=50)

        if self.tip_pose_topic:
            rospy.Subscriber(self.tip_pose_topic, PoseStamped, self._on_tip_pose, queue_size=200)
        if self.image_topic:
            rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=3)

        # Start/stop triggers (same as your pipeline)
        rospy.Subscriber("/com_3d/log_start", Empty, self._on_start, queue_size=1)
        rospy.Subscriber("/com_3d/log_stop", Empty, self._on_stop, queue_size=1)

        # Timer for estimation
        rospy.Timer(rospy.Duration(1.0 / max(self.est_update_hz, 1e-3)), self._on_est_timer)

        rospy.loginfo("[ft_clock_logger_streaming_v2] ready. fs=%.1fHz, decimate=%d, est_update=%.1fHz",
                      self.est.fs_hz, self.est.decimate, self.est_update_hz)

    def _make_paths(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = self.base_name.strip() or f"exp_{ts}"
        outdir = self.output_dir.strip()
        if not outdir:
            try:
                rp = rospkg.RosPack()
                pkg_path = rp.get_path("com_3d")
                outdir = os.path.join(pkg_path, "experiments")
            except Exception:
                outdir = os.path.join(os.path.expanduser("~"), "experiments")
        os.makedirs(outdir, exist_ok=True)
        csv_path = os.path.join(outdir, f"{base}.csv")
        vid_path = os.path.join(outdir, f"{base}.mp4")
        return csv_path, vid_path

    def _open_csv(self, csv_path):
        f = open(csv_path, "w", newline="")
        w = csv.writer(f)
        header = [
            "time",
            "fx","fy","fz",
            "tx","ty","tz",
            "ee_x","ee_y","ee_z",
            "tag_qx","tag_qy","tag_qz","tag_qw",
            "contact","trigger"
        ]
        w.writerow(header)
        f.flush()
        return f, w

    def _on_start(self, _msg):
        with self.csv_lock:
            if self.recording:
                return
            csv_path, vid_path = self._make_paths()
            self.csv_f, self.csv_w = self._open_csv(csv_path)
            self.recording = True
            self.last_flush = rospy.Time.now().to_sec()
            self.est.reset()
        rospy.loginfo("[ft_clock_logger_streaming_v2] START logging -> %s", csv_path)

    def _on_stop(self, _msg):
        with self.csv_lock:
            self.recording = False
            if self.csv_f is not None:
                try:
                    self.csv_f.flush()
                    self.csv_f.close()
                except Exception:
                    pass
            self.csv_f = None
            self.csv_w = None

        # Print last estimate on stop
        if self.est.last_est is not None:
            m, zc, ths, _, n = self.est.last_est
            rospy.loginfo("[ft_clock_logger_streaming_v2] STOP: last_est m=%.4f kg, zc=%.4f m, theta*=%.2f deg (n=%d)",
                          m, zc, np.rad2deg(ths), n)
        else:
            rospy.logwarn("[ft_clock_logger_streaming_v2] STOP: no estimate available (need more samples / tags).")

    def _on_fw_contact(self, msg: Bool):
        self.ft_contact_flag = 1 if msg.data else 0

    def _on_fw_trigger(self, msg: Bool):
        self.ft_trigger_flag = 1 if msg.data else 0

    def _on_tag(self, msg: AprilTagDetectionArray):
        # Keep the most recent tag quaternion (xyzw). Use first detection by default.
        if msg is None or len(msg.detections) == 0:
            return
        det = msg.detections[0]
        q = det.pose.pose.pose.orientation
        with self.csv_lock:
            self.tag_latest = {"tq": np.array([q.x, q.y, q.z, q.w], dtype=float)}

    def _on_tip_pose(self, msg: PoseStamped):
        p = msg.pose.position
        with self.csv_lock:
            self.tip_pose_latest = np.array([p.x, p.y, p.z], dtype=float)

    def _on_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self.csv_lock:
            self.last_image = img

    def _get_tip_pose(self):
        with self.csv_lock:
            ee = None if self.tip_pose_latest is None else self.tip_pose_latest.copy()
        return ee

    def _on_ft(self, msg: WrenchStamped):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        ft = (float(msg.wrench.force.x),  float(msg.wrench.force.y),  float(msg.wrench.force.z),
              float(msg.wrench.torque.x), float(msg.wrench.torque.y), float(msg.wrench.torque.z))

        with self.csv_lock:
            recording = self.recording
            csv_w = self.csv_w
            csv_f = self.csv_f
            tag = self.tag_latest
            contact = self.ft_contact_flag
            trigger = self.ft_trigger_flag

        if not recording or csv_w is None:
            return

        ee = self._get_tip_pose()
        if ee is None:
            # if you don't have tip_pose_topic wired, you can still log NaNs
            ee = np.array([np.nan, np.nan, np.nan], dtype=float)

        tq = tag["tq"] if tag is not None else np.array([np.nan, np.nan, np.nan, np.nan], dtype=float)

        # CSV write
        with self.csv_lock:
            if not self.recording or self.csv_w is None:
                return
            self.csv_w.writerow([t, *ft, ee[0], ee[1], ee[2], tq[0], tq[1], tq[2], tq[3], int(contact), int(trigger)])
            now = rospy.Time.now().to_sec()
            if (now - self.last_flush) >= self.flush_period and self.csv_f is not None:
                self.csv_f.flush()
                self.last_flush = now

        # Estimator ingest
        if tag is not None and np.isfinite(tq).all():
            self.est.add_sample(
                f_xyz=np.array(ft[0:3]),
                ee_xyz=ee,
                tag_quat_xyzw=tq,
                contact_flag=contact,
                trigger_flag=trigger
            )

    def _on_est_timer(self, _evt):
        est = self.est.update_estimate()
        if est is None:
            return
        m, zc, ths, _, _ = est
        self.est_pub.publish(Float64MultiArray(data=[m, zc, ths]))

        if self.publish_debug and self.est.last_debug is not None:
            dbg = self.est.last_debug
            th = dbg["th_deg"]
            tau_y = dbg["tau_y"]
            tau_fit_y = dbg["tau_fit_y"]
            N = int(len(th))
            if N >= 5:
                payload = [float(N)] + th.tolist() + tau_y.tolist() + tau_fit_y.tolist() + [
                    float(dbg["theta_star_deg"]), float(dbg["m"]), float(dbg["zc"])
                ]
                self.dbg_pub.publish(Float64MultiArray(data=payload))


if __name__ == "__main__":
    FTClockLogger()
    rospy.spin()