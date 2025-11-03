#!/usr/bin/env python3
import os, csv, rospy, rospkg
from threading import Lock
from std_msgs.msg import Empty
import tf2_ros
import tf2_py
from geometry_msgs.msg import TransformStamped

FLUSH_PERIOD = 0.25  # seconds

class EEPoseCSVLogger:
    def __init__(self):
        # Params
        self.run_base   = rospy.get_param('~run_base', None)
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.ee_frame   = rospy.get_param('~ee_frame',   'finger_tip')   # or 'ee_link' if that’s your chain tip
        self.rate_hz    = float(rospy.get_param('~rate_hz', 100.0)) # timer-based logging rate

        if not self.run_base:
            rospy.logfatal("[ee_pose_csv_logger] Missing ~run_base")
            rospy.signal_shutdown("missing ~run_base")
            return

        # Output path (match your FT logger convention)
        pkg_path = rospkg.RosPack().get_path('com_3d')
        self.out_dir = os.path.join(pkg_path, 'experiments'); os.makedirs(self.out_dir, exist_ok=True)

        # State
        self.traj_idx   = 0
        self.recording  = False
        self.csv_path   = None
        self.csv_f      = None
        self.csv_w      = None
        self.csv_lock   = Lock()
        self.last_flush = rospy.Time.now().to_sec()

        # TF2
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf)

        # Triggers
        rospy.Subscriber('/com_3d/log_start', Empty, self._on_log_start, queue_size=1)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._on_log_stop,  queue_size=1)

        # Timer-based logging (decoupled from /tf pub cadence)
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate_hz), self._on_timer)

        rospy.loginfo("[ee_pose_csv_logger] run_base=%s out_dir=%s base=%s ee=%s rate=%.1f Hz",
                      self.run_base, self.out_dir, self.base_frame, self.ee_frame, self.rate_hz)

    def _on_log_start(self, _):
        if self.recording: return
        self.traj_idx += 1
        stem = f"{self.run_base}_t{self.traj_idx:03d}_ee"
        self.csv_path = os.path.join(self.out_dir, stem + ".csv")
        self.csv_f = open(self.csv_path, 'w', newline='')
        self.csv_w = csv.writer(self.csv_f)
        # pose as xyz + quaternion; also include rpy_deg for convenience
        self.csv_w.writerow(['ros_time_sec','x','y','z','qx','qy','qz','qw','roll_deg','pitch_deg','yaw_deg'])
        self.recording = True
        self.last_flush = rospy.Time.now().to_sec()
        rospy.loginfo("[ee_pose_csv_logger] START -> %s", self.csv_path)

    def _on_log_stop(self, _):
        if not self.recording: return
        self.recording = False
        if self.csv_f:
            self.csv_f.flush(); self.csv_f.close()
            self.csv_f = None; self.csv_w = None
        rospy.loginfo("[ee_pose_csv_logger] STOP (disarm)")

    @staticmethod
    def _quat_to_rpy_deg(qx, qy, qz, qw):
        # minimal / no external deps: convert quaternion->RPY (XYZ) then to degrees
        import math
        # Source: standard quaternion to Euler (roll-pitch-yaw) XYZ
        sinr_cosp = 2*(qw*qx + qy*qz); cosr_cosp = 1 - 2*(qx*qx + qy*qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2*(qw*qy - qz*qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi/2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2*(qw*qz + qx*qy); cosy_cosp = 1 - 2*(qy*qy + qz*qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll*180.0/math.pi, pitch*180.0/math.pi, yaw*180.0/math.pi)

    def _on_timer(self, _evt):
        if not self.recording or self.csv_w is None: return
        # Get the latest transform (now)
        try:
            ts: TransformStamped = self.tfbuf.lookup_transform(
                self.base_frame, self.ee_frame, rospy.Time(0), rospy.Duration(0.05)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return

        t = ts.header.stamp.to_sec() if ts.header.stamp else rospy.Time.now().to_sec()
        x = ts.transform.translation.x
        y = ts.transform.translation.y
        z = ts.transform.translation.z
        qx = ts.transform.rotation.x
        qy = ts.transform.rotation.y
        qz = ts.transform.rotation.z
        qw = ts.transform.rotation.w
        r_deg, p_deg, y_deg = self._quat_to_rpy_deg(qx,qy,qz,qw)

        with self.csv_lock:
            self.csv_w.writerow([f"{t:.6f}",
                                 f"{x:.9f}", f"{y:.9f}", f"{z:.9f}",
                                 f"{qx:.9f}", f"{qy:.9f}", f"{qz:.9f}", f"{qw:.9f}",
                                 f"{r_deg:.5f}", f"{p_deg:.5f}", f"{y_deg:.5f}"])
            now = rospy.Time.now().to_sec()
            if (now - self.last_flush) >= FLUSH_PERIOD:
                self.csv_f.flush()
                self.last_flush = now

    def shutdown(self):
        self._on_log_stop(None)

if __name__ == '__main__':
    rospy.init_node('ee_pose_csv_logger')
    node = EEPoseCSVLogger()
    rospy.on_shutdown(getattr(node, 'shutdown', lambda: None))
    rospy.spin()
