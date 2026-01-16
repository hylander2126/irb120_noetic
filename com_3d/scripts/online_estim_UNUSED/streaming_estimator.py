#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Empty
from geometry_msgs.msg import WrenchStamped, PoseStamped
from threading import Lock
from collections import deque

from scipy.signal import butter, sosfilt, sosfilt_zi
from scipy.optimize import curve_fit

from apriltag_ros.msg import AprilTagDetectionArray

from com_3d.com_estimation import tau_app_model, tau_model
from com_3d.helper_fns import quat_normalize, quat_conj, quat_mul, quat_to_rotvec


class CausalLPF:
    """
    Causal (real-time) Low-Pass Filter combining median filtering and Butterworth IIR (inf impulse response) filtering.
    """
    def __init__(self, fs_hz: float, cutoff_hz: float, order: int, dim: int, medwin: int):
        self.dim = int(dim)
        self.medwin = max(1, int(medwin))
        self._hist = deque(maxlen=self.medwin) # for median filt
        self.sos = butter(int(order), float(cutoff_hz), fs=float(fs_hz), btype="low", output="sos") # build butter
        self._init_zi() # init filter internal state

    def _init_zi(self):
        zi0 = sosfilt_zi(self.sos) # SOS (second-order sections) initial state
        self.zi = np.tile(zi0[:, :, None], (1, 1, self.dim))

    def step(self, x):
        x = np.asarray(x, dtype=float).reshape(self.dim) # Add new sample
        self._hist.append(x)
        x_med = np.median(np.stack(self._hist, axis=0), axis=0) # get median (robsut to spikes)
        y, self.zi = sosfilt(self.sos, x_med.reshape(1, self.dim), zi=self.zi, axis=0) # SOS butter LPF
        return y.reshape(self.dim)


class SlidingEstimator:
    """
    Collect samples and recursively fit mass and zc to torque using a sliding window approach.
    """
    def __init__(self):
        self.lock = Lock()
        self.fs_hz = float(rospy.get_param('~fs_hz', 500.0))
        self.decimate = int(rospy.get_param('~est_decimate', 5))
        self.window = int(rospy.get_param('~est_window_size', 250))
        self.min_samples = int(rospy.get_param('~min_samples', 60))

        # Relative Orientation Tracking
        self.q_ref = None # reference quaternion at start
        self.is_active = False # only accumulate samples between start/stop

        # Filtering
        self.force_filt = CausalLPF(self.fs_hz, cutoff_hz=5.0, order=4, dim=3, medwin=5) # force filter
        self.rv_filt = CausalLPF(self.fs_hz, cutoff_hz=5.0, order=4, dim=3, medwin=5) # rotation vector filter

        # Buffers
        self._t = deque(maxlen=self.window)  # timestamps
        self._th = deque(maxlen=self.window) # tilt angle samples
        self._tau_s = deque(maxlen=self.window) # projected scalar torque samples
        self._fx = deque(maxlen=self.window) # force x samples
        self._eez = deque(maxlen=self.window) # end-effector z samples
        self._sample_i = 0 # sample counter for decimation
        self.t_start = 0.0

    def start(self, q_initial):
        with self.lock:
            self.q_ref = quat_normalize(np.array(q_initial, float)) # reference orientation (init) normalized
            self._t.clear(); self._th.clear(); self._tau_s.clear(); self._fx.clear(); self._eez.clear() # clear buffers
            self.is_active = True # start accumulating
            self.t_start = rospy.get_time()

            # Read the runtime parameters (avoid TypeError by converting to np arrays)
            self.o_obj = np.array(rospy.get_param('/com_3d/o_obj', [0.66, 0, 0]), dtype=float)
            self.rc0 = np.array(rospy.get_param('/com_3d/rc0_known', [-0.05, 0, 0]), dtype=float)
            self.e_hat = np.array(rospy.get_param('~e_hat', [0.0, 1.0, 0.0]), dtype=float)
            self.e_hat /= (np.linalg.norm(self.e_hat) + 1e-12)
            
            # Compute perpendicular distance from rc0 to axis e_hat (distance from axis line in plane orthog to e_hat)
            # d = ||rc0 - (rc0 . e_hat) e_hat||
            self.d = float(np.linalg.norm(self.rc0 - np.dot(self.rc0, self.e_hat) * self.e_hat))
        rospy.loginfo("[SlidingEstimator] Resetting buffers. q_ref set.")

    def stop(self):
        with self.lock:
            self.is_active = False

    def add_sample(self, f_xyz, ee_xyz, q_xyzw):
        """ 
        Add a new sample (force, end-effector position, orientation) on every
        force callback. Internally decimates to reduce computation.
        """
        if not self.is_active or self.q_ref is None: return

        f_f = self.force_filt.step(f_xyz) # filter the force (filt adds to internal buffer)
        
        # Relative orientation math
        q_norm_curr = quat_normalize(np.array(q_xyzw, float))
        q_rel = quat_mul(quat_conj(self.q_ref), q_norm_curr) # get relative quat to initial
        
        # Convert q_rel to rotation vector
        rv = quat_to_rotvec(q_rel, normalize=False)
        rv_f = self.rv_filt.step(rv) # filter the rotvec (filt adds to internal buffer)

        self._sample_i += 1
        if (self._sample_i % self.decimate) != 0: # decimate (1 sample per 'decimate' updates)
            return

        theta = float(np.linalg.norm(rv_f)) # tilt angle (rad)
        
        rf = np.asarray(ee_xyz, float) - self.o_obj # compute lever arm
        f_app = -np.asarray(f_f, float) # applied force (negate sensor reading)
        
        tau_vec = np.asarray(tau_app_model(f_app.reshape(1,3), rf.reshape(1,3)), float).reshape(-1)[:3] # compute applied torque
        tau_s = float(np.dot(tau_vec, self.e_hat)) # project torque onto e_hat

        t_rel = rospy.get_time() - self.t_start

        with self.lock:
            self._t.append(t_rel)
            self._th.append(theta)
            self._tau_s.append(tau_s)
            self._fx.append(float(f_f[0]))
            self._eez.append(float(ee_xyz[2]))

    def fit(self):
        """
        Fit mass and zc to the collected samples in the sliding window. Runs at slower rate than add_sample.
        Returns (m_est, zc_est, theta_star, debug_dict) or None if not enough samples.
        """
        with self.lock:
            if len(self._th) < self.min_samples: 
                return None
            t_arr, th, tau_s, fx, eez = np.array(self._t), np.array(self._th), np.array(self._tau_s), np.array(self._fx), np.array(self._eez)

        # Define bounds explicitly (to clamp lin guess too)
        b_min = [0.01, 0.01] # mass, zc
        b_max = [10.0, 1.0]  # mass, zc

        # Initial guesses based on linear fit (to improve convergence) HEURISTIC
        m0, z0 = 0.5, 0.1
        if len(th) >= 10: # if enough data, fit fx ~= a*theta + b
            a, b = np.polyfit(th, fx, 1) # (first order aka linear)
            theta_star_guess = -b/a if abs(a) > 1e-6 else 0 # if slope too shallow, tan(theta) ~0 to z0 explodes
            
            # Only use heuristic if produces physically sane values
            if 0.01 < theta_star_guess < 1.5:
                z0 = self.d / np.tan(theta_star_guess)
                m0 = abs(a) * np.median(eez) / (9.81 * z0)
        
        # CLAMP guesses to bounds
        m0 = np.clip(m0, b_min[0] + 1e-6, b_max[0] - 1e-6)
        z0 = np.clip(z0, b_min[1] + 1e-6, b_max[1] - 1e-6)

        def fit_fn(theta, m, zc):
            t = tau_model(theta, m, zc, rc0_known=self.rc0, e_hat=self.e_hat).reshape(-1, 3) # returns Nx3
            return t @ self.e_hat # scalar projection

        try:
            popt, _ = curve_fit(fit_fn, th, tau_s, p0=[m0, z0], 
                                bounds=(b_min, b_max), maxfev=1000)
            m_est, zc_est = float(popt[0]), float(popt[1])
            theta_star = float(np.arctan2(self.d, zc_est + 1e-12)) # avoid div by zero
            
            # Build debug dict (for plotting, etc.)
            y_fit = fit_fn(th, m_est, zc_est)
            dbg = {"time": t_arr, 
                   "th": th, 
                   "tau": tau_s, 
                   "fit": y_fit, 
                   "ths": theta_star, 
                   "m": m_est, 
                   "z": zc_est}    
            return m_est, zc_est, theta_star, dbg
        
        except Exception as e: 
            rospy.logwarn(f"[streaming_estimator] Fit failed: {e}, z0={z0}, m0={m0}")
            return None


class EstimatorNode:
    def __init__(self):

        rospy.init_node('streaming_estimator')

        self.est = SlidingEstimator()
        self.last_q, self.last_ee = None, None

        # Logic: Start accumulation ONLY on log_start if we have a valid tag
        def on_start(_):
            if self.last_q is not None: 
                self.est.start(self.last_q)
            else: 
                rospy.logwarn("[streaming_estimator] Waiting for AprilTag to start...")

        rospy.Subscriber('/com_3d/log_start', Empty, on_start)
        rospy.Subscriber('/com_3d/log_stop',  Empty, lambda _: self.est.stop())
        rospy.Subscriber('/netft_data_transformed', WrenchStamped, self._on_ft)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._on_tag)
        rospy.Subscriber('/com_3d/tip_pose_fk', PoseStamped, self._on_ee)

        self.pub = rospy.Publisher('/com_3d/online_estimate', Float64MultiArray, queue_size=1)
        self.pub_dbg = rospy.Publisher('/com_3d/online_fit_debug', Float64MultiArray, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self._on_timer)

    def _on_tag(self, msg):
        if msg.detections:
            q = msg.detections[0].pose.pose.pose.orientation
            self.last_q = [q.x, q.y, q.z, q.w]

    def _on_ee(self, msg):
        p = msg.pose.position
        self.last_ee = [p.x, p.y, p.z]

    def _on_ft(self, msg):
        if self.last_q and self.last_ee:
            self.est.add_sample([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z], 
                                self.last_ee, self.last_q)
            # self.est.add_sample([msg.wrench.force.z, msg.wrench.force.x, msg.wrench.force.y], # CRTICIAL: MUST REORDER AXES TO MATCH ROBOT FRAME

    def _on_timer(self, _):
        if not self.est.is_active:
            return
        out = self.est.fit()
        if out:
            m, z, ts, d = out
            self.pub.publish(Float64MultiArray(data=[m, z, ts]))

            # Serialize: N. then arrays of length N
            N = len(d["th"])
            self.pub_dbg.publish(Float64MultiArray(data=[float(N)] + list(d["time"]) + list(d["th"]) + 
                                 list(d["tau"]) + list(d["fit"]) + [d["ths"], m, z])) # Publish dict as flat array

if __name__ == '__main__':
    EstimatorNode()
    rospy.spin()