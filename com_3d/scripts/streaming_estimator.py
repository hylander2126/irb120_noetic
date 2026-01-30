#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Empty, Bool
from geometry_msgs.msg import WrenchStamped, PoseStamped
from threading import Lock
from scipy.signal import butter, filtfilt
from scipy.optimize import curve_fit
from scipy.stats import linregress
import os

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
        self.e_hat /= (np.linalg.norm(self.e_hat) + 1e-12) # normalize  + stability
        # d = perp distance from rc0 to axis
        self.d = float(np.linalg.norm(self.rc0 - np.dot(self.rc0, self.e_hat) * self.e_hat))

    def start(self, q_initial):
        """
        Snapshot params, set ref quat, clear buffers, start collection.
        """
        with self.lock:
            self.update_params()
            self.q_ref = quat_normalize(np.array(q_initial, float))
            
            # Clear buffers
            self.times, self.forces, self.ee_pos, self.quats, self.contacts, self.retracts = [], [], [], [], [], []
            self.t_start = rospy.get_time()
            self.is_collecting = True
        rospy.loginfo("[BatchEstimator] Collection STARTED.")

    def add_sample(self, f_xyz, ee_xyz, q_xyzw, contact_active, retract_active):
        """
        Log time, force, ee pos, quat, contact/retract status.
        """
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

    def _estimate_params(self, th, tau_flat, tag="phase"):
        """
        Fit (m, zc) for one phase using full 3-vector torque (flattened).
        Returns dict with params + residual score.
        """
        th = np.asarray(th, float).reshape(-1)
        tau_vec = np.asarray(tau_flat, float).reshape(-1, 3) # ensure (N, 3)
        tau_proj = tau_vec @ self.e_hat  # (N,)

        ## ==== Linear seed (cheap) ====
        slope, intercept, *_ = linregress(th, tau_proj)

        ths0 = float(np.clip(-intercept / (slope + 1e-12), 1e-2, np.pi/2)) # CLIP ths to reasonable range (~zero to 90 deg)
        z0 = float(np.clip(abs(self.d / np.tan(ths0)), 0.02, 1.0)) # zc from theta* CLIP to [2cm, 1m]
        m0 = float(np.clip(abs(slope) / (9.81 * max(z0, 1e-3)), 0.05, 20.0)) # m in [0.05kg, 20kg]

        ## ==== Outlier rejection based on lin guess ====
        outlier_k = 2.5 # MAD sigma threshold (experimental mean absolute deviation)
        resid = tau_proj - (slope * th + intercept)
        
        med = np.median(resid)
        mad = np.median(np.abs(resid - med))
        sigma = 1.4826 * mad + 1e-12
        inliers = np.abs(resid - med) <= (outlier_k * sigma)

        # keep only inliers (don't care about outliers at all)
        th_in = th[inliers]
        tau_in = tau_vec[inliers]          # (Ni,3)

        ## ==== Nonlinear curve fit ====
        def _fit_target(th_in, m, zc): # (wrapper cause curve_fit doesn't like extra args)
            return tau_model(th_in, m, zc, rc0_known=self.rc0, e_hat=self.e_hat)

        popt, _ = curve_fit(
            _fit_target,
            th_in,
            tau_in.reshape(-1), # ensure raveled / flattened
            p0=[m0, z0],
            bounds=([0.01, 0.01], [20.0, 1.0]),
            maxfev=4000
        )
        m_est, zc_est = map(float, popt)

        # Residual score for weighting
        tau_fit = _fit_target(th_in, m_est, zc_est).reshape(-1)
        resid_nl = (tau_in - tau_fit).reshape(-1)
        # mean squared residual per element
        mse = float(np.mean(resid_nl**2))

        ths_est = float(np.arctan2(self.d, zc_est))

        return {
            "m": m_est,
            "zc": zc_est,
            "ths": ths_est,
            "mse": mse,
            # Strictly inliers only
            "th_inlier": th_in,
            "tau_inlier": tau_in,
        }

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

        # -----------------------------------------------------------
        # 1. PROCESS & FILTER 
        # -----------------------------------------------------------
        f_filt = filtfilt(self.b, self.a, f_raw, axis=0) # low-pass filter force
        
        # Calculate Angle (Theta)
        rot_vecs = []
        q_ref_conj = quat_conj(self.q_ref)
        for q in q_raw:
            q_norm = quat_normalize(q)
            q_rel = quat_mul(q_ref_conj, q_norm) 
            rv = quat_to_rotvec(q_rel, normalize=False)
            rot_vecs.append(rv)
        
        rot_vecs = np.array(rot_vecs)
        rot_vecs_filt = filtfilt(self.b, self.a, rot_vecs, axis=0) # low-pass filter rot vec TODO maybe calculate e_hat here?
        theta_exp = np.linalg.norm(rot_vecs_filt, axis=1) # TODO: test filtering on scalar theta instead

        # -----------------------------------------------------------
        # 2. MASKING for CONTACT and RETRACT DATA
        # -----------------------------------------------------------
        push_mask = c_raw & (~r_raw)    # Push phase only
        retr_mask = c_raw & (r_raw)   # Retract phase only

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
        def _get_raveled_tapp(ee_pos, force):
            return tau_app_model(-force, ee_pos-self.o_obj) # Raveled array (N*3,) (F_APP FLIPS THE MEASURED FORCE)
        tau_push = _get_raveled_tapp(ee_push, f_push)        
        tau_retr = _get_raveled_tapp(ee_retr, f_retr)

        # -----------------------------------------------------------
        # 4. FIT TWICE, COMBINE
        # -----------------------------------------------------------
        try:
            # Get fits for each phase of pushing
            fit_push = self._estimate_params(th_push, tau_push, tag="push")
            fit_retr = self._estimate_params(th_retr, tau_retr, tag="retract")
            
            # Combine weighted by residuals
            # m_est, zc_est, ths_est, w_push, w_retr = self._combine_two_fits(fit_push, fit_retr)
            eps = 1e-12
            w_push = 1.0 / (fit_push["mse"] + eps)
            w_retr = 1.0 / (fit_retr["mse"] + eps)
            wsum = w_push + w_retr

            m_est = (w_push * fit_push["m"] + w_retr * fit_retr["m"]) / wsum
            zc_est = (w_push * fit_push["zc"] + w_retr * fit_retr["zc"]) / wsum
            ths_est = float(np.arctan2(self.d, zc_est))
    

            # WRITE SUMMARY TO FILE
            # -------------------------------------------------------
            log_stem = rospy.get_param("/com_3d/current_log_stem", None)
            out_dir = rospy.get_param("/com_3d/current_log_dir",  ".")

            path = os.path.join(out_dir, log_stem + "_fit_summary.txt") if log_stem else None
            try:
                with open(path, "a") as f:
                    f.write(f"Batch Fit Results:\n")
                    f.write(f"  Push Phase:    m={fit_push['m']:.3f} kg, zc={fit_push['zc']:.3f} m, th*={np.rad2deg(fit_push['ths']):.2f} deg, MSE={fit_push['mse']:.4f}\n")
                    f.write(f"  Retract Phase: m={fit_retr['m']:.3f} kg, zc={fit_retr['zc']:.3f} m, th*={np.rad2deg(fit_retr['ths']):.2f} deg, MSE={fit_retr['mse']:.4f}\n")
                    f.write(f"  Weights: push={w_push:.4f}, retract={w_retr:.4f}\n")
                    f.write(f"  COMBINED:      m={m_est:.3f} kg, zc={zc_est:.3f} m, th*={np.rad2deg(ths_est):.2f} deg\n")
                    f.write("\n")
                rospy.loginfo(f"[BatchEstimator] Fit summary written to: {path}")
            except Exception as e:
                rospy.logwarn(f"[BatchEstimator] Could not write fit summary to file: {e}")
            

            # -------------------------------------------------------
            # 6. PLOTTING DATA PREP (inliers only, plotter gets raw data itself)
            # -------------------------------------------------------
            th_p = fit_push["th_inlier"]
            tau_p_vec = fit_push["tau_inlier"]
            th_r = fit_retr["th_inlier"]
            tau_r_vec = fit_retr["tau_inlier"]

            # Project to scalar for easy plotting (Project onto e_hat)
            tau_p = tau_p_vec @ self.e_hat
            tau_r = tau_r_vec @ self.e_hat

            # Per-phase fit curves (using each phase's own params)
            fit_p = (tau_model(th_p, fit_push["m"], fit_push["zc"], rc0_known=self.rc0, e_hat=self.e_hat).reshape(-1, 3) @ self.e_hat) # (Np,)
            fit_r = (tau_model(th_r, fit_retr["m"], fit_retr["zc"], rc0_known=self.rc0, e_hat=self.e_hat).reshape(-1, 3) @ self.e_hat) # (Nr,)

            # Optional downsample to keep messages small
            # TODO: implement this

            plot_object = {
                "Np": len(th_p),
                "Nr": len(th_r),
                "th_p": th_p,
                "tau_p": tau_p,
                "fit_p": fit_p,
                "th_r": th_r,
                "tau_r": tau_r,
                "fit_r": fit_r,
                "ths": ths_est,
                "m": m_est,
                "z": zc_est,
            }
            return m_est, zc_est, ths_est, plot_object # Return for publishing
        
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
            
            Np = int(d["Np"])
            Nr = int(d["Nr"])

            flat_data = [float(Np), float(Nr)] \
                + list(d["th_p"]) + list(d["tau_p"]) + list(d["fit_p"]) \
                + list(d["th_r"]) + list(d["tau_r"]) + list(d["fit_r"]) \
                + [d["ths"], m, z]
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