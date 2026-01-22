#!/usr/bin/env python3
"""streaming_estimator_node_v3.py

Streaming (true online) estimator for (m, zc, theta_star) using your existing
com_3d.com_estimation models:

  tau_app_model(F, rf)            -> ravel (3N,)
  tau_model(theta, m, zc, rc0_known, e_hat=[0,1,0]) -> ravel (3N,)

This node:
- Subscribes to FT wrench, tip pose (EE xyz), and AprilTag detections (quat)
- Causally filters force + rotvec (median + SOS butter LPF)
- Decimates samples for fitting (e.g., 500Hz -> 100Hz)
- Runs curve_fit at a low rate (e.g., 5Hz) over a sliding window
- Publishes /com_3d/online_estimate as Float64MultiArray [m, zc, theta_star_rad]

IMPORTANT: rc0_known and o_obj are read from the parameter server EVERY update,
so vel_move can set them per-object during runtime:
  /com_3d/o_obj      [x,y,z]
  /com_3d/rc0_known  [x,y,0]   (your best 2D CoM projection)

"""

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped, PoseStamped
from threading import Lock
from collections import deque

try:
    from scipy.signal import butter, sosfilt, sosfilt_zi
    from scipy.optimize import curve_fit
except Exception:
    butter = sosfilt = sosfilt_zi = curve_fit = None

try:
    from apriltag_ros.msg import AprilTagDetectionArray
except Exception:
    AprilTagDetectionArray = None

try:
    from com_3d.com_estimation import tau_app_model, tau_model
except Exception:
    tau_app_model = tau_model = None


class CausalLPF:
    """Streaming median + causal Butterworth LPF (SOS) for vector signals."""
    def __init__(self, fs_hz: float, cutoff_hz: float, order: int, dim: int, medwin: int):
        self.dim = int(dim)
        self.medwin = max(1, int(medwin))
        self._hist = deque(maxlen=self.medwin)

        self.sos = None
        self.zi = None

        if butter is not None and cutoff_hz and cutoff_hz > 0:
            self.sos = butter(int(order), float(cutoff_hz), fs=float(fs_hz), btype="low", output="sos")
            self._init_zi()

    def _init_zi(self):
        # For x shaped (n_samples, n_channels) with axis=0,
        # scipy expects zi shaped (n_sections, 2, n_channels).
        zi0 = sosfilt_zi(self.sos)              # (n_sections, 2)
        self.zi = np.tile(zi0[:, :, None],      # (n_sections, 2, 1)
                          (1, 1, self.dim))     # (n_sections, 2, dim)

    def reset(self):
        self._hist.clear()
        if self.sos is not None:
            self._init_zi()
        else:
            self.zi = None

    def step(self, x):
        x = np.asarray(x, dtype=float).reshape(self.dim)
        if not np.isfinite(x).all():
            return None

        self._hist.append(x)
        x_med = np.median(np.stack(self._hist, axis=0), axis=0)

        if self.sos is None:
            return x_med.copy()

        if self.zi is None:
            self._init_zi()

        y, self.zi = sosfilt(self.sos, x_med.reshape(1, self.dim), zi=self.zi, axis=0)
        return y.reshape(self.dim)


def quat_xyzw_from_detection(msg):#: 'AprilTagDetectionArray'):
    # pick first detection if present
    if msg is None or len(msg.detections) == 0:
        return None
    det = msg.detections[0]
    try:
        q = det.pose.pose.pose.orientation
    except Exception:
        return None
    return np.array([q.x, q.y, q.z, q.w], dtype=float)


def rotvec_from_quat_xyzw(q):
    # Minimal axis-angle from quaternion; safe + fast.
    q = np.asarray(q, dtype=float).reshape(4)
    if not np.isfinite(q).all():
        return None
    n = np.linalg.norm(q)
    if n < 1e-12:
        return None
    q = q / n
    x, y, z, w = q
    # clamp w
    w = float(np.clip(w, -1.0, 1.0))
    angle = 2.0 * np.arccos(w)
    s = np.sqrt(max(1e-16, 1.0 - w*w))
    axis = np.array([x, y, z], dtype=float) / s
    # rotvec = axis * angle
    return axis * angle


class SlidingEstimator:
    def __init__(self):
        self.lock = Lock()

        self.fs_hz = float(rospy.get_param('~fs_hz', 500.0))
        self.decimate = int(rospy.get_param('~est_decimate', 5))
        self.window = int(rospy.get_param('~est_window_size', 250))
        self.min_samples = int(rospy.get_param('~min_samples', max(30, self.window // 4)))

        # Filtering
        self.force_filt = CausalLPF(
            fs_hz=self.fs_hz,
            cutoff_hz=float(rospy.get_param('~force_cutoff_hz', 5.0)),
            order=int(rospy.get_param('~force_order', 4)),
            dim=3,
            medwin=int(rospy.get_param('~force_medwin', 5))
        )
        self.rv_filt = CausalLPF(
            fs_hz=self.fs_hz,
            cutoff_hz=float(rospy.get_param('~rotvec_cutoff_hz', 5.0)),
            order=int(rospy.get_param('~rotvec_order', 4)),
            dim=3,
            medwin=int(rospy.get_param('~rotvec_medwin', 5))
        )

        # Buffers (decimated)
        self._th = deque(maxlen=self.window)          # (N,)
        self._axis = deque(maxlen=self.window)        # (N,3)
        self._tau = deque(maxlen=self.window)         # (N,3)
        self._rf = deque(maxlen=self.window)          # (N,3)
        # Also keep a scalar torque about the expected tipping axis e_hat.
        # This matches the way you plot/assess torque in analyze_experiment (draw_axes=[1]).
        self._tau_s = deque(maxlen=self.window)       # (N,)
        self._rf = deque(maxlen=self.window)          # (N,3)

        # For online initial guesses (mirrors your offline linear-guess trick)
        self._fx = deque(maxlen=self.window)          # (N,) measured fx
        self._eez = deque(maxlen=self.window)         # (N,) ee_z (push height)

        self._sample_i = 0

    def _read_runtime_params(self):
        # Read every update so vel_move can change per object
        o_obj = np.array(rospy.get_param('/com_3d/o_obj', [0.0, 0.0, 0.0]), dtype=float).reshape(3)
        rc0 = np.array(rospy.get_param('/com_3d/rc0_known', [0.05, 0.0, 0.0]), dtype=float).reshape(3)
        # Tip axis: default matches your current analyze_experiment override
        e_hat = np.array(rospy.get_param('~e_hat', [0.0, 1.0, 0.0]), dtype=float).reshape(3)
        en = np.linalg.norm(e_hat)
        if en < 1e-9:
            e_hat = np.array([0.0, 1.0, 0.0])
        else:
            e_hat = e_hat / en
        return o_obj, rc0, e_hat

    def add_sample(self, f_xyz, ee_xyz, q_xyzw):
        if tau_app_model is None:
            return

        f_f = self.force_filt.step(f_xyz)
        if f_f is None:
            return

        rv = rotvec_from_quat_xyzw(q_xyzw)
        if rv is None:
            return
        rv_f = self.rv_filt.step(rv)
        if rv_f is None:
            return

        self._sample_i += 1
        if (self._sample_i % max(self.decimate, 1)) != 0:
            return

        theta = float(np.linalg.norm(rv_f))
        axis = rv_f / (theta + 1e-12)

        o_obj, rc0, e_hat = self._read_runtime_params()

        rf = np.asarray(ee_xyz, dtype=float).reshape(3) - o_obj

        # IMPORTANT: match your offline sign convention
        f_app = -np.asarray(f_f, dtype=float).reshape(3)

        # Use shapes (N,3) to match your docstring expectations.
        tau_vec = np.asarray(tau_app_model(f_app.reshape(1,3), rf.reshape(1,3)), dtype=float).reshape(-1)[:3]
        tau_s = float(np.dot(tau_vec, e_hat))

        with self.lock:
            self._th.append(theta)
            self._axis.append(axis)
            self._tau.append(tau_vec)
            self._rf.append(rf)
            # keep raw-ish quantities for initial guess
            self._fx.append(float(f_f[0]))
            self._eez.append(float(np.asarray(ee_xyz, dtype=float).reshape(3)[2]))
            self._tau_s.append(tau_s)

    def fit(self):
        if curve_fit is None or tau_model is None or tau_app_model is None:
            return None

        with self.lock:
            if len(self._th) < self.min_samples:
                return None
            th = np.asarray(self._th, dtype=float)
            # tau = np.asarray(self._tau, dtype=float)   # (N,3)
            tau_s = np.asarray(self._tau_s, dtype=float)
            fx = np.asarray(self._fx, dtype=float)
            eez = np.asarray(self._eez, dtype=float)

        o_obj, rc0, e_hat = self._read_runtime_params()

        # # Flatten to match analyze_experiment usage (3N,)
        # y = tau.reshape(-1)
         # ------------------------------------------------------------
        # Fit on *scalar* torque about e_hat (NOT full 3-vector).
        # This matches how you evaluate torque in analyze_experiment (draw_axes=[1])
        # and avoids the other two noisy/irrelevant components dominating the fit.
        # ------------------------------------------------------------
        y = tau_s  # (N,)

        def fit_fn(theta, m, zc):
            # return tau_model(theta, m, zc, rc0_known=rc0, e_hat=e_hat)
            # tau_model returns ravel(3N,). Convert to (N,3) and project onto e_hat.
            t = tau_model(theta, m, zc, rc0_known=rc0, e_hat=e_hat).reshape(-1, 3)
            return t @ e_hat

        # ------------------------------------------------------------
        # Online initial guess from the same trick you use offline:
        # linear fit of F_x vs theta -> theta_star_guess, zc_guess, m_guess.
        # See analyze_experiment.py: you p0=[m_calc, zc_calc] from that linear fit.
        # ------------------------------------------------------------
        # Distance of planar rc0_known to the tipping axis (generalizes "abs(com_gt[0])")
        d = float(np.linalg.norm(rc0 - np.dot(rc0, e_hat) * e_hat))
        
        # m0 = float(rospy.get_param('~m_guess', 1.0))
        # z0 = float(rospy.get_param('~zc_guess', 1.0))

        # Defaults (used when the linear guess is unstable)
        m0_default = float(rospy.get_param('~m_guess', 0.5))
        z0_default = float(rospy.get_param('~zc_guess', 0.10))

        m0, z0 = m0_default, z0_default
        theta_star_guess = None

        # Robust-ish linear fit y = a*theta + b
        if len(th) >= 10:
            thm = float(np.mean(th))
            fxm = float(np.mean(fx))
            denom = float(np.sum((th - thm) ** 2))
            if denom > 1e-9:
                a = float(np.sum((th - thm) * (fx - fxm)) / denom)
                b = fxm - a * thm
                if abs(a) > 1e-6:
                    theta_star_guess = -b / a

        if theta_star_guess is not None and np.isfinite(theta_star_guess):
            # keep it in a sane range (0 to < 90deg)
            if 1e-3 < theta_star_guess < (0.5 * np.pi - 1e-2):
                # zc_guess from geometry: z = d / tan(theta*)
                zc_guess = abs(d) / max(1e-6, np.tan(theta_star_guess))
                if np.isfinite(zc_guess) and zc_guess > 1e-4:
                    z0 = float(zc_guess)
                    # m_guess uses the same scaling you used offline:
                    # m ≈ |slope| * z_push / (g * zc)
                    z_push = float(np.median(eez)) if len(eez) else 0.10
                    g = 9.8067
                    m_guess = abs(a) * abs(z_push) / max(1e-6, (g * z0))
                    if np.isfinite(m_guess) and m_guess > 1e-4:
                        m0 = float(m_guess)


        # bounds = (
        #     [float(rospy.get_param('~m_min', 0.001)), float(rospy.get_param('~zc_min', 0.001))],
        #     [float(rospy.get_param('~m_max', 10.0)),  float(rospy.get_param('~zc_max', 1.0))],
        # )
        # ------------------------------------------------------------
        # Bounds: keep user-configured global bounds, but ALSO tighten around
        # the online guess when it is available, so the fit doesn't slam to extremes
        # when the early data is not informative.
        # ------------------------------------------------------------
        m_min_g = float(rospy.get_param('~m_min', 0.05))
        m_max_g = float(rospy.get_param('~m_max', 5.0))
        z_min_g = float(rospy.get_param('~zc_min', 0.01))
        z_max_g = float(rospy.get_param('~zc_max', 1.0))

        if (m0 != m0_default) or (z0 != z0_default):
            m_min = max(m_min_g, 0.2 * m0)
            m_max = min(m_max_g, 5.0 * m0)
            z_min = max(z_min_g, 0.2 * z0)
            z_max = min(z_max_g, 5.0 * z0)
        else:
            m_min, m_max, z_min, z_max = m_min_g, m_max_g, z_min_g, z_max_g


        # try:
        #     popt, _ = curve_fit(fit_fn, th, y, p0=[m0, z0], bounds=bounds, maxfev=3000)
        # except Exception:
        #     return None

        # m_est, zc_est = float(popt[0]), float(popt[1])
        # theta_star = float(np.arctan2(np.linalg.norm(rc0[0:2]), zc_est))
        # return m_est, zc_est, theta_star

        # --- Robustify bounds + initial guess ---
        # It's easy for early-window regression to yield a huge z0 (or tiny m0),
        # which can invert bounds (z_min > z_max) or make p0 infeasible.
        eps_m = 1e-6
        eps_z = 1e-6

        # If tightened bounds are invalid, fall back to global bounds.
        if not (np.isfinite(m_min) and np.isfinite(m_max)) or (m_max <= m_min + eps_m):
            m_min, m_max = m_min_g, m_max_g
        if not (np.isfinite(z_min) and np.isfinite(z_max)) or (z_max <= z_min + eps_z):
            z_min, z_max = z_min_g, z_max_g

        # If bounds are still too tight for a strict interior point, widen a hair.
        if (m_max - m_min) <= 2.0 * eps_m:
            mid = 0.5 * (m_min + m_max)
            m_min, m_max = max(m_min_g, mid - 1e-3), min(m_max_g, mid + 1e-3)
        if (z_max - z_min) <= 2.0 * eps_z:
            mid = 0.5 * (z_min + z_max)
            z_min, z_max = max(z_min_g, mid - 1e-3), min(z_max_g, mid + 1e-3)

        # Ensure p0 is feasible (strictly inside bounds)
        if not np.isfinite(m0):
            m0 = 0.5 * (m_min + m_max)
        if not np.isfinite(z0):
            z0 = 0.5 * (z_min + z_max)

        m0 = float(np.clip(m0, m_min + eps_m, m_max - eps_m))
        z0 = float(np.clip(z0, z_min + eps_z, z_max - eps_z))

        try:
            popt, _ = curve_fit(
                fit_fn,
                th,
                y,
                p0=[m0, z0],
                bounds=([m_min, z_min], [m_max, z_max]),
                maxfev=4000,
            )
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[streaming_estimator] curve_fit failed: {e}")
            return None

        m_est, zc_est = float(popt[0]), float(popt[1])
        theta_star = float(np.arctan2(abs(d), zc_est + 1e-12))

        # Also return debug series for live plotting
        y_fit = np.asarray(fit_fn(th, m_est, zc_est), dtype=float)
        dbg = {
            "theta_deg": np.rad2deg(th),
            "tau_app": y,
            "tau_fit": y_fit,
            "theta_star_deg": np.rad2deg(theta_star),
            "m0": m0,
            "z0": z0,
        }

        return m_est, zc_est, theta_star, dbg


class EstimatorNode:
    def __init__(self):
        rospy.init_node('streaming_estimator', anonymous=False)

        self.ft_topic = rospy.get_param('~ft_stream_topic', '/netft_data_transformed')
        self.tag_topic = rospy.get_param('~tag_topic', '/tag_detections')
        self.tip_pose_topic = rospy.get_param('~tip_pose_topic', '/com_3d/tip_pose_fk')

        self.update_hz = float(rospy.get_param('~est_update_hz', 5.0))

        self.est = SlidingEstimator()
        self.last_q = None
        self.last_ee = None

        self.pub = rospy.Publisher('/com_3d/online_estimate', Float64MultiArray, queue_size=1)
        self.pub_dbg = rospy.Publisher('/com_3d/online_fit_debug', Float64MultiArray, queue_size=1)

        rospy.Subscriber(self.ft_topic, WrenchStamped, self._on_ft, queue_size=1000)
        if AprilTagDetectionArray is not None:
            rospy.Subscriber(self.tag_topic, AprilTagDetectionArray, self._on_tag, queue_size=10)
        rospy.Subscriber(self.tip_pose_topic, PoseStamped, self._on_tip_pose, queue_size=50)

        rospy.Timer(rospy.Duration(1.0 / max(self.update_hz, 0.1)), self._on_timer)

        rospy.loginfo('[streaming_estimator] up: ft=%s tag=%s tip_pose=%s', self.ft_topic, self.tag_topic, self.tip_pose_topic)

    def _on_tag(self, msg):
        q = quat_xyzw_from_detection(msg)
        if q is not None:
            self.last_q = q

    def _on_tip_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.last_ee = np.array([p.x, p.y, p.z], dtype=float)

    def _on_ft(self, msg: WrenchStamped):
        if self.last_q is None or self.last_ee is None:
            return
        w = msg.wrench
        f = np.array([w.force.x, w.force.y, w.force.z], dtype=float)
        self.est.add_sample(f, self.last_ee, self.last_q)

    def _on_timer(self, _evt):
        out = self.est.fit()
        if out is None:
            return
        m_est, zc_est, theta_star, dbg = out
        msg = Float64MultiArray()
        msg.data = [m_est, zc_est, theta_star]
        self.pub.publish(msg)

        # Debug series for live plotting: [theta_deg... , tau_app..., tau_fit..., theta_star_deg]
        try:
            dd = Float64MultiArray()
            N = len(dbg["theta_deg"])
            dd.data = [float(N)] + \
                      list(map(float, dbg["theta_deg"])) + \
                      list(map(float, dbg["tau_app"])) + \
                      list(map(float, dbg["tau_fit"])) + \
                      [float(dbg["theta_star_deg"]), float(m_est), float(zc_est)]
            self.pub_dbg.publish(dd)
        except Exception:
            pass


if __name__ == '__main__':
    if tau_app_model is None or tau_model is None:
        print('[streaming_estimator] ERROR: could not import com_3d.com_estimation (tau_app_model/tau_model).')
    elif curve_fit is None or sosfilt is None:
        print('[streaming_estimator] ERROR: scipy not available (need scipy.signal + scipy.optimize).')
    else:
        EstimatorNode()
        rospy.spin()
