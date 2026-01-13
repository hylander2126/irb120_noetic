#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Empty
from geometry_msgs.msg import WrenchStamped, PoseStamped
from threading import Lock
from collections import deque
from apriltag_ros.msg import AprilTagDetectionArray
from com_3d.com_estimation import tau_app_model, tau_model
from scipy.optimize import curve_fit

# --- Quat Math Helpers ---
def quat_norm(q):
    return q / (np.linalg.norm(q) + 1e-12)

def quat_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]], float)

def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], float)

def rotvec_from_quat(q):
    q = quat_norm(q)
    w = float(np.clip(q[3], -1.0, 1.0))
    angle = 2.0 * np.arccos(w)
    s = np.sqrt(max(1e-16, 1.0 - w*w))
    return (q[:3] / s) * angle

class AccumulatingEstimator:
    def __init__(self):
        self.lock = Lock()
        self.is_active = False
        self.q_ref = None
        
        # Buffers
        self._th, self._tau_s, self._fx, self._eez = [], [], [], []
        self.f_median_buf = deque(maxlen=7) # Median "cheatcode" filter
        self._sample_i = 0
        self.decimate = int(rospy.get_param('~est_decimate', 5))
        
        # Caching params
        self.o_obj, self.rc0 = None, None
        self.e_hat = np.array([0.0, 1.0, 0.0])
        self.lever_arm_pivot = 0.0

    def reset_and_start(self, initial_q):
        with self.lock:
            # Fetch and convert to float numpy arrays to avoid TypeErrors
            self.o_obj = np.array(rospy.get_param('/com_3d/o_obj', [0.66, 0, 0]), dtype=float)
            self.rc0 = np.array(rospy.get_param('/com_3d/rc0_known', [-0.05, 0, 0]), dtype=float)
            self.lever_arm_pivot = float(np.linalg.norm(self.rc0 - np.dot(self.rc0, self.e_hat) * self.e_hat))
            
            self.q_ref = quat_norm(initial_q)
            self._th, self._tau_s, self._fx, self._eez = [], [], [], []
            self.f_median_buf.clear()
            self.is_active = True
        rospy.loginfo("[streaming_estimator] BUFFER STARTED. Orientation relative to start pose.")

    def stop(self, _):
        with self.lock:
            self.is_active = False
        rospy.loginfo("[streaming_estimator] BUFFER STOPPED.")

    def add_sample(self, f_xyz, ee_xyz, q_curr):
        if not self.is_active: return

        self._sample_i += 1
        if (self._sample_i % self.decimate) != 0: return

        # 1. Median filter FT sensor noise
        self.f_median_buf.append(f_xyz)
        if len(self.f_median_buf) < 7: return
        f_smooth = np.median(self.f_median_buf, axis=0)

        # 2. Relative Rotation
        q_rel = quat_mul(quat_conj(self.q_ref), quat_norm(q_curr))
        rv = rotvec_from_quat(q_rel)
        theta = float(np.linalg.norm(rv))

        # 3. Physics model Torque projection
        rf = np.asarray(ee_xyz) - self.o_obj
        tau_vec = np.cross(rf, -np.asarray(f_smooth)) 
        tau_s = float(np.dot(tau_vec, self.e_hat))

        with self.lock:
            self._th.append(theta)
            self._tau_s.append(tau_s)
            self._fx.append(float(f_smooth[0]))
            self._eez.append(float(ee_xyz[2]))

    def fit(self):
        with self.lock:
            if len(self._th) < 30: return None
            th, tau_s = np.array(self._th), np.array(self._tau_s)
        
        def fit_fn(theta, m, zc):
            t = tau_model(theta, m, zc, rc0_known=self.rc0, e_hat=self.e_hat).reshape(-1, 3)
            return t @ self.e_hat

        try:
            popt, _ = curve_fit(fit_fn, th, tau_s, p0=[0.5, 0.1], 
                                bounds=([0.01, 0.01], [10.0, 1.0]), maxfev=400)
            m_est, zc_est = float(popt[0]), float(popt[1])
            ts_rad = float(np.arctan2(self.lever_arm_pivot, zc_est))
            return m_est, zc_est, ts_rad, {"th": th, "tau": tau_s, "fit": fit_fn(th, *popt)}
        except Exception: return None

class EstimatorNode:
    def __init__(self):
        rospy.init_node('streaming_estimator')
        self.est = AccumulatingEstimator()
        self.last_q, self.last_ee = None, None

        # Logic: We can't start the estimator until we have a 'last_q' for reference
        def on_log_start(_):
            if self.last_q is None:
                rospy.logwarn("[streaming_estimator] Cannot start: No AprilTag detection yet!")
            else:
                self.est.reset_and_start(self.last_q)

        rospy.Subscriber('/com_3d/log_start', Empty, on_log_start)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self.est.stop)
        
        rospy.Subscriber(rospy.get_param('~ft_stream_topic'), WrenchStamped, self._on_ft)
        rospy.Subscriber(rospy.get_param('~tag_topic'), AprilTagDetectionArray, self._on_tag)
        rospy.Subscriber(rospy.get_param('~tip_pose_topic'), PoseStamped, self._on_tip_pose)

        self.pub = rospy.Publisher('/com_3d/online_estimate', Float64MultiArray, queue_size=1)
        self.pub_dbg = rospy.Publisher('/com_3d/online_fit_debug', Float64MultiArray, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self._on_timer)

    def _on_tag(self, msg):
        if msg.detections:
            q = msg.detections[0].pose.pose.pose.orientation
            self.last_q = [q.x, q.y, q.z, q.w]

    def _on_tip_pose(self, msg):
        p = msg.pose.position
        self.last_ee = [p.x, p.y, p.z]

    def _on_ft(self, msg):
        if self.last_q and self.last_ee:
            self.est.add_sample([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z], 
                                 self.last_ee, self.last_q)
        else:
            # Diagnostic for why data isn't accumulating
            rospy.logwarn_throttle(2.0, f"[streaming_estimator] Waiting for: {'Tag ' if not self.last_q else ''}{'EE ' if not self.last_ee else ''}")

    def _on_timer(self, _):
        res = self.est.fit()
        if res:
            m, z, ts, d = res
            self.pub.publish(Float64MultiArray(data=[m, z, ts]))
            N = len(d["th"])
            # Packet: [N, theta_deg, tau_app, tau_fit, ts_deg, m, z]
            self.pub_dbg.publish(Float64MultiArray(data=[N] + list(np.rad2deg(d["th"])) + 
                                 list(d["tau"]) + list(d["fit"]) + [np.rad2deg(ts), m, z]))

if __name__ == '__main__':
    EstimatorNode()
    rospy.spin()