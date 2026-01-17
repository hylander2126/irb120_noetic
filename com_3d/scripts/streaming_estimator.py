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
            
            self.t_start = rospy.get_time()
            self.is_collecting = True
        rospy.loginfo("[BatchEstimator] Collection STARTED.")

    def add_sample(self, f_xyz, ee_xyz, q_xyzw, contact_active):
        if not self.is_collecting:
            return
            
        with self.lock:
            t_now = rospy.get_time() - self.t_start
            self.times.append(t_now)
            self.forces.append(f_xyz)
            self.ee_pos.append(ee_xyz)
            self.quats.append(q_xyzw)
            self.contacts.append(contact_active)

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

        rospy.loginfo(f"[BatchEstimator] Processing batch of {len(time)} samples...")

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
        # 2. MASKING (Using ForceWatcher Status)
        # -----------------------------------------------------------
        mask = c_raw

        # Fallback: If FW never triggers (comms issue or contact window too short), us conservative mask from F mag.
        if np.sum(mask) < 10:
            fmag = np.linalg.norm(f_filt, axis=1)
            # Baseline from the first 0.5s of data (or first 250 samples)
            n0 = int(min(len(fmag), max(50, 0.5 * self.fs_hz)))
            f0 = float(np.median(fmag[:n0])) if n0 > 0 else float(np.median(fmag))
            # Require force meaningfully above baseline and some rotation
            mask_f = (fmag > (f0 + 0.15)) & (theta_exp > 0.01)
            if np.sum(mask_f) >= 25:
                rospy.logwarn(
                    "[BatchEstimator] ForceWatcher mask too small (%d). Falling back to force-based mask (%d).",
                    int(np.sum(mask)), int(np.sum(mask_f))
                )
                mask = mask_f
            else:
                rospy.logwarn(
                    "[BatchEstimator] ForceWatcher never reported a usable contact window (mask=%d) and fallback mask was also too small (%d). Fitting aborted.",
                    int(np.sum(mask)), int(np.sum(mask_f))
                )
                return None
            
            
        # Extract Trimmed Data
        f_trim  = f_filt[mask]
        th_trim = theta_exp[mask]
        ee_trim = ee_raw[mask]
        t_trim  = time[mask]
        
        rospy.loginfo(f"[BatchEstimator] Fitting on {len(t_trim)} samples identified by ForceWatcher.")

        # -----------------------------------------------------------
        # 3. CALCULATE TORQUE
        # -----------------------------------------------------------
        rf = ee_trim - self.o_obj
        f_app = -f_trim  # Reaction -> Action
        tau_app_trim = tau_app_model(f_app, rf) # Flattened array (N*3)
        
        # -----------------------------------------------------------
        # 4. INITIAL GUESS (Linear Regression)
        # -----------------------------------------------------------
        try:
            # Fit force X vs Theta
            lin_slope, lin_b, _, _, _ = linregress(th_trim, f_trim[:, 0])
            
            theta_star_guess = -lin_b / lin_slope if abs(lin_slope) > 1e-4 else 0
            if not (0.01 < theta_star_guess < 1.0):
                theta_star_guess = 0.1 
                
            zc_calc = abs(self.d / np.tan(theta_star_guess))
            m_calc = abs(abs(lin_slope) * np.mean(ee_trim[:,2]) / (9.81 * zc_calc))
            
            # Clamp Init
            zc_calc = np.clip(zc_calc, 0.05, 0.8)
            m_calc  = np.clip(m_calc, 0.1, 8.0)
            
        except:
            m_calc, zc_calc = 0.5, 0.2 

        # -----------------------------------------------------------
        # 5. NON-LINEAR CURVE FIT
        # -----------------------------------------------------------
        def fit_target(th, m, zc):
            return tau_model(th, m, zc, rc0_known=self.rc0, e_hat=self.e_hat)

        try:
            popt, _ = curve_fit(
                fit_target, 
                th_trim, 
                tau_app_trim, 
                p0=[m_calc, zc_calc], 
                bounds=([0.01, 0.01], [20.0, 1.0]),
                maxfev=2000
            )
            m_est, zc_est = popt
            theta_star_est = float(np.arctan2(self.d, zc_est))
            
            rospy.loginfo(f"[BatchEstimator] Result: m={m_est:.3f}, zc={zc_est:.3f}, th*={np.rad2deg(theta_star_est):.2f}")
            
            # -------------------------------------------------------
            # 6. DEBUG DATA
            # -------------------------------------------------------
            step = max(1, len(t_trim) // 100)
            
            # Compute fit curve for plotting
            tau_fit_flat = fit_target(th_trim, m_est, zc_est)
            
            # Project to scalar for easy plotting (Project onto e_hat)
            tau_vec_exp = tau_app_trim.reshape(-1, 3)
            tau_vec_fit = tau_fit_flat.reshape(-1, 3)
            
            tau_scalar_exp = np.dot(tau_vec_exp, self.e_hat)
            tau_scalar_fit = np.dot(tau_vec_fit, self.e_hat)

            dbg = {
                "time": t_trim[::step],
                "th": th_trim[::step],
                "tau": tau_scalar_exp[::step],
                "fit": tau_scalar_fit[::step],
                "ths": theta_star_est,
                "m": m_est,
                "z": zc_est
            }
            return m_est, zc_est, theta_star_est, dbg

        except Exception as e:
            rospy.logwarn(f"[BatchEstimator] Curve fit failed: {e}")
            return None


class EstimatorNode:
    def __init__(self):
        rospy.init_node('streaming_estimator')
        self.est = BatchEstimator()
        self.last_q, self.last_ee = None, None
        self.contact_active = False # Track latest ForceWatcher status

        # --- SUBSCRIBERS ---
        rospy.Subscriber('/com_3d/log_start', Empty, self._on_start)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._on_stop)
        rospy.Subscriber('/netft_data_transformed', WrenchStamped, self._on_ft)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self._on_tag)
        rospy.Subscriber('/com_3d/tip_pose_fk', PoseStamped, self._on_ee)
        
        # Subscribe to ForceWatcher status for masking
        rospy.Subscriber('/com_3d/fw_contact_status', Bool, self._on_contact)

        # --- PUBLISHERS ---
        self.pub_res = rospy.Publisher('/com_3d/online_estimate', Float64MultiArray, queue_size=1, latch=False)
        self.pub_dbg = rospy.Publisher('/com_3d/online_fit_debug', Float64MultiArray, queue_size=1, latch=True)

    def _on_contact(self, msg):
        self.contact_active = msg.data

    def _on_start(self, _):
        if self.last_q is not None:
            self.est.start(self.last_q)
        else:
            rospy.logwarn("[EstimatorNode] Cannot start: No AprilTag orientation received yet.")

    def _on_stop(self, _):
        out = self.est.stop_and_fit()
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
                self.contact_active
            )

if __name__ == '__main__':
    EstimatorNode()
    rospy.spin()