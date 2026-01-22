#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Empty, Bool
from geometry_msgs.msg import WrenchStamped, PoseStamped
from threading import Lock
from scipy.signal import butter, filtfilt
from scipy.optimize import curve_fit
from scipy.stats import linregress

from apriltag_ros.msg import AprilTagDetectionArray
from com_3d.com_estimation import tau_app_model, tau_model
from com_3d.helper_fns import quat_normalize, quat_conj, quat_mul, quat_to_rotvec

class BatchEstimator:
    """
    Accumulates data during a push, then performs a one-shot batch fit 
    using the ForceWatcher status to mask valid tipping data.
    """
    def __init__(self):
        self.lock = Lock()
        self.fs_hz = float(rospy.get_param('~fs_hz', 500.0))
        
        # Buffer storage
        self.times = []
        self.forces = [] # [fx, fy, fz]
        self.ee_pos = [] # [x, y, z]
        self.quats = []  # [x, y, z, w]
        self.contacts = [] # [bool] from ForceWatcher
        self.retracts = [] # [bool] from retract status
        
        # State
        self.is_collecting = False
        self.q_ref = None
        self.t_start = 0.0

        # Filter params (4th order, 5Hz)
        self.b, self.a = butter(4, 5.0, fs=self.fs_hz, btype='low')

        self.update_params()

    def update_params(self):
        self.o_obj = np.array(rospy.get_param('/com_3d/o_obj', [0.66, 0, 0]), dtype=float)
        self.rc0 = np.array(rospy.get_param('/com_3d/rc0_known', [-0.05, 0, 0]), dtype=float)
        self.e_hat = np.array(rospy.get_param('~e_hat', [0.0, 1.0, 0.0]), dtype=float)
        self.e_hat /= (np.linalg.norm(self.e_hat) + 1e-12)
        # d = perp distance from rc0 to axis
        self.d = float(np.linalg.norm(self.rc0 - np.dot(self.rc0, self.e_hat) * self.e_hat))

    def start(self, q_initial):
        with self.lock:
            self.update_params()
            self.q_ref = quat_normalize(np.array(q_initial, float))
            
            # Clear buffers
            self.times = []
            self.forces = []
            self.ee_pos = []
            self.quats = []
            self.contacts = []
            self.retracts = []
            self.t_start = rospy.get_time()
            self.is_collecting = True
        rospy.loginfo("[BatchEstimator] Collection STARTED.")

    def add_sample(self, f_xyz, ee_xyz, q_xyzw, contact_active, retract_active):
        if not self.is_collecting:
            return
            
        with self.lock:
            t_now = rospy.get_time() - self.t_start
            self.times.append(t_now)
            self.forces.append(f_xyz)
            self.ee_pos.append(ee_xyz)
            self.quats.append(q_xyzw)
            self.contacts.append(contact_active)
            self.retracts.append(retract_active)


    def _fit_phase(self, th, tau_flat, tag="phase"):
        """
        Fit (m, zc) for one phase using full 3-vector torque (flattened).
        Returns dict with params + residual score.
        """
        th = np.asarray(th, float).reshape(-1)
        tau_flat = np.asarray(tau_flat, float).reshape(-1)  # (N*3,)

        if th.size < 15:
            raise RuntimeError(f"Not enough samples for {tag}: {th.size}")

        # --- Initial guess via linear fit on the *projected* torque (for guess only)
        tau_vec = tau_flat.reshape(-1, 3)
        tau_proj = tau_vec @ self.e_hat  # (N,)

        # Robust-ish linear guess near small angles:
        # use first chunk (low theta) if possible
        idx = np.argsort(th)
        th_s = th[idx]
        tp_s = tau_proj[idx]

        k = min(len(th_s), 60)
        th_g = th_s[:k]
        tp_g = tp_s[:k]

        # If theta range is tiny, fall back
        if (np.max(th_g) - np.min(th_g)) < 1e-3:
            m0, z0 = 0.5, 0.2
        else:
            slope, intercept, _, _, _ = linregress(th_g, tp_g)

            # crude theta* guess: when torque crosses ~0 (depends on your sign conventions)
            # keep it bounded/sane
            if abs(slope) > 1e-6:
                ths0 = float(np.clip(-intercept / slope, 0.02, 1.2))
            else:
                ths0 = 0.2

            # zc from theta*
            z0 = float(np.clip(abs(self.d / np.tan(ths0)), 0.02, 1.0))

            # m guess: scale from slope magnitude; this is heuristic
            # (better than nothing; curve_fit will refine)
            m0 = float(np.clip(abs(slope) / (9.81 * max(z0, 1e-3)), 0.05, 20.0))

        def _fit_target(th_in, m, zc):
            return tau_model(th_in, m, zc, rc0_known=self.rc0, e_hat=self.e_hat)

        popt, _ = curve_fit(
            _fit_target,
            th,
            tau_flat,
            p0=[m0, z0],
            bounds=([0.01, 0.01], [20.0, 1.0]),
            maxfev=4000
        )
        m_est, zc_est = map(float, popt)

        # Residual score for weighting
        tau_fit = _fit_target(th, m_est, zc_est).reshape(-1)
        resid = tau_flat - tau_fit
        # mean squared residual per element
        mse = float(np.mean(resid**2))

        ths_est = float(np.arctan2(self.d, zc_est))

        return {
            "m": m_est,
            "zc": zc_est,
            "ths": ths_est,
            "mse": mse,
        }
    
    def _combine_two_fits(self, fit_push, fit_retr):
        """
        Residual-weighted combine. Lower MSE => higher weight.
        """
        eps = 1e-12
        w1 = 1.0 / (fit_push["mse"] + eps)
        w2 = 1.0 / (fit_retr["mse"] + eps)
        wsum = w1 + w2

        m = (w1 * fit_push["m"] + w2 * fit_retr["m"]) / wsum
        zc = (w1 * fit_push["zc"] + w2 * fit_retr["zc"]) / wsum

        ths = float(np.arctan2(self.d, zc))
        return float(m), float(zc), ths, float(w1), float(w2)
    

    def stop_and_fit(self):
        """
        Triggered on stop. Processes the full batch and returns estimate.
        """
        with self.lock:
            self.is_collecting = False
            if len(self.times) < 50:
                rospy.logwarn("[BatchEstimator] Not enough samples collected.")
                return None

            # Convert to numpy arrays
            time = np.array(self.times)
            f_raw = np.array(self.forces)
            ee_raw = np.array(self.ee_pos)
            q_raw = np.array(self.quats)
            c_raw = np.array(self.contacts, dtype=bool)
            r_raw = np.array(self.retracts, dtype=bool)
        rospy.loginfo(f"[BatchEstimator] Processing batch of {len(time)} samples...")

        # DEBUG BLOCK:
        rospy.loginfo(f"[BatchEstimator] RAW DATA CHECK:")
        rospy.loginfo(f"  Total samples: {len(time)}")
        rospy.loginfo(f"  Contact True count: {np.sum(c_raw)} / {len(c_raw)}")
        rospy.loginfo(f"  Retract True count: {np.sum(r_raw)} / {len(r_raw)}")
        rospy.loginfo(f"  Contact samples: {c_raw[:20]}...")  # First 20
        rospy.loginfo(f"  Retract samples: {r_raw[:20]}...")  # First 20

        # -----------------------------------------------------------
        # 1. PROCESS & FILTER 
        # -----------------------------------------------------------
        f_filt = filtfilt(self.b, self.a, f_raw, axis=0)
        
        # Calculate Angle (Theta)
        rot_vecs = []
        q_ref_conj = quat_conj(self.q_ref)
        for q in q_raw:
            q_norm = quat_normalize(q)
            q_rel = quat_mul(q_ref_conj, q_norm) 
            rv = quat_to_rotvec(q_rel, normalize=False)
            rot_vecs.append(rv)
        
        rot_vecs = np.array(rot_vecs)
        rot_vecs_filt = filtfilt(self.b, self.a, rot_vecs, axis=0)
        theta_exp = np.linalg.norm(rot_vecs_filt, axis=1)

        # -----------------------------------------------------------
        # 2. MASKING for CONTACT and RETRACT DATA
        # -----------------------------------------------------------
        mask_contact = c_raw # Contact-based mas (ForceWatcher)
        push_mask = mask_contact & (~r_raw)    # Push phase only
        retr_mask = mask_contact & (r_raw)   # Retract phase only

        n_push = int(np.sum(push_mask))
        n_retr = int(np.sum(retr_mask))

        # Fallback: If FW never triggers (comms issue or contact window too short), ABORT
        if n_push < 15 or n_retr < 15:
            rospy.logwarn(
                "[BatchEstimator] Not enough usable samples after masking: (push mask=%d, retr mask=%d). Aborting.",
                n_push, n_retr
            )
            return None
        
        def _trim(m):
            return f_filt[m], theta_exp[m], ee_raw[m], time[m]
        
        # Extract Trimmed Data
        f_push, th_push, ee_push, t_push = _trim(push_mask)
        f_retr, th_retr, ee_retr, t_retr = _trim(retr_mask)

        
        rospy.loginfo(f"[BatchEstimator] Fitting on {len(t_push)} samples from push phase and {len(t_retr)} samples from retract phase.")

        # -----------------------------------------------------------
        # 3. CALCULATE TORQUE
        # -----------------------------------------------------------
        rf = ee_push - self.o_obj
        f_app = -f_push  # Reaction -> Action
        tau_push = tau_app_model(f_app, rf) # Raveled array (N*3,)

        rf = ee_retr - self.o_obj
        f_app = -f_retr  # Reaction -> Action
        tau_retr = tau_app_model(f_app, rf) # Raveled array (N*3,)

        # -----------------------------------------------------------
        # 4. FIT TWICE, COMBINE
        # -----------------------------------------------------------
        try:
            # First use push data for fitting
            fit_push = self._fit_phase(th_push, tau_push, tag="push")
            fit_retr = self._fit_phase(th_retr, tau_retr, tag="retract")
            
            m_est, zc_est, ths_est, w_push, w_retr = self._combine_two_fits(fit_push, fit_retr)

            rospy.loginfo(f"[BatchEstimator] Fit Results:")
            rospy.loginfo(f"  Push Phase:    m={fit_push['m']:.3f} kg, zc={fit_push['zc']:.3f} m, th*={np.rad2deg(fit_push['ths']):.2f} deg, MSE={fit_push['mse']:.4f}")
            rospy.loginfo(f"  Retract Phase:  m={fit_retr['m']:.3f} kg, zc={fit_retr['zc']:.3f} m, th*={np.rad2deg(fit_retr['ths']):.2f} deg, MSE={fit_retr['mse']:.4f}")
            rospy.loginfo(f"  Weights: push={w_push:.4f}, retract={w_retr:.4f}")
            rospy.loginfo(f"  COMBINED:      m={m_est:.3f} kg, zc={zc_est:.3f} m, th*={np.rad2deg(ths_est):.2f} deg")
            
            # -------------------------------------------------------
            # 6. DEBUG DATA (combine push + retr for plotting)
            # Fit curve uses combined params
            # -------------------------------------------------------
            # For debug, concatenate push + retr data (not for fitting)
            th_all = np.concatenate([th_push, th_retr])
            tau_all = np.concatenate([tau_push.reshape(-1, 3), tau_retr.reshape(-1, 3)], axis=0)
            t_all = np.concatenate([t_push, t_retr])

            # Sort by theta for nice plot
            idx_sort = np.argsort(th_all)
            th_all = th_all[idx_sort]
            tau_all = tau_all[idx_sort]
            t_all = t_all[idx_sort]

            # Model prediction at those theta points
            tau_fit_all = tau_model(th_all, m_est, zc_est, rc0_known=self.rc0, e_hat=self.e_hat)

            # Project to scalar for easy plotting (Project onto e_hat)
            tau_scalar_exp = tau_all @ self.e_hat
            tau_scalar_fit = tau_fit_all.reshape(-1, 3) @ self.e_hat

            step = max(1, len(t_all) // 120)
            dbg = {
                "time": np.arange(len(th_all))[::step] / max(self.fs_hz, 1.0),  # just a monotonic x-axis for debug
                "th": th_all[::step],
                "tau": tau_scalar_exp[::step],
                "fit": tau_scalar_fit[::step],
                "ths": ths_est,
                "m": m_est,
                "z": zc_est
            }
            return m_est, zc_est, ths_est, dbg
        
        except Exception as e:
            rospy.logwarn(f"[BatchEstimator] Separate fit failed: {e}")
            return None


class EstimatorNode:
    def __init__(self):
        rospy.init_node('streaming_estimator')
        self.est = BatchEstimator()
        self.last_q, self.last_ee = None, None
        self.contact_active = False # Track latest ForceWatcher status
        self.retract_active = False # Track retract status

        # --- SUBSCRIBERS ---
        rospy.Subscriber('/com_3d/est_start', Empty, self._on_start)
        rospy.Subscriber('/com_3d/est_stop',  Empty, self._on_stop)
        rospy.Subscriber('/netft_data_transformed', WrenchStamped, self._on_ft)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._on_tag)
        rospy.Subscriber('/com_3d/tip_pose_fk', PoseStamped, self._on_ee)
        
        # Subscribe to ForceWatcher status for masking
        rospy.Subscriber('/com_3d/fw_contact_status', Bool, self._on_contact)
        rospy.Subscriber('/com_3d/retract_phase', Bool, self._on_retract)

        # --- PUBLISHERS ---
        self.pub_res = rospy.Publisher('/com_3d/online_estimate', Float64MultiArray, queue_size=1, latch=False)
        self.pub_dbg = rospy.Publisher('/com_3d/online_fit_debug', Float64MultiArray, queue_size=1, latch=True)

    def _on_contact(self, msg):
        self.contact_active = msg.data

    def _on_retract(self, msg: Bool):
        # On retract, trigger retract flag so our fit knows when to use push vs retract data
        self.retract_active = msg.data

    def _on_start(self, _):
        self.retract_active = False  # Reset retract status on new start (optional, good hygiene)
        # Don't need to reset contact because that is handled by ForceWatcher...
        if self.last_q is not None:
            self.est.start(self.last_q)
        else:
            rospy.logwarn("[EstimatorNode] Cannot start: No AprilTag orientation received yet.")

    def _on_stop(self, _):
        out = self.est.stop_and_fit()
        self.retract_active = False  # Reset retract status on stop (optional, good hygiene)
        if out:
            m, z, ts, d = out
            self.pub_res.publish(Float64MultiArray(data=[m, z, ts]))
            
            N = len(d["th"])
            flat_data = [float(N)] + \
                        list(d["time"]) + \
                        list(d["th"]) + \
                        list(d["tau"]) + \
                        list(d["fit"]) + \
                        [d["ths"], m, z]
            self.pub_dbg.publish(Float64MultiArray(data=flat_data))

    def _on_tag(self, msg):
        if msg.detections:
            q = msg.detections[0].pose.pose.pose.orientation
            self.last_q = [q.x, q.y, q.z, q.w]

    def _on_ee(self, msg):
        p = msg.pose.position
        self.last_ee = [p.x, p.y, p.z]

    def _on_ft(self, msg):
        if self.last_q and self.last_ee:
            # Pass contact status along with data
            self.est.add_sample(
                [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z], 
                self.last_ee, 
                self.last_q,
                self.contact_active,
                self.retract_active
            )

if __name__ == '__main__':
    EstimatorNode()
    rospy.spin()