#!/usr/bin/env python3
import os, csv, rospy, rospkg
from threading import Lock
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty

FLUSH_PERIOD = 0.25  # seconds

class FTForceCSVLogger:
    def __init__(self):
        self.run_base = rospy.get_param('~run_base', None)
        if not self.run_base:
            rospy.logfatal("[ft_force_csv_logger] Missing ~run_base")
            rospy.signal_shutdown("missing ~run_base")
            return

        self.ft_topic = rospy.get_param('~ft_topic', '/netft_data_transformed')
        pkg_path = rospkg.RosPack().get_path('com_3d')
        self.out_dir = os.path.join(pkg_path, 'experiments'); os.makedirs(self.out_dir, exist_ok=True)

        self.traj_idx = 0
        self.recording = False
        self.csv_path = None
        self.csv_f = None
        self.csv_w = None
        self.csv_lock = Lock()
        self.last_flush = rospy.Time.now().to_sec()

        rospy.Subscriber(self.ft_topic, WrenchStamped, self._on_ft, queue_size=50)
        rospy.Subscriber('/com_3d/log_start', Empty, self._on_log_start, queue_size=1)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._on_log_stop,  queue_size=1)

        rospy.loginfo("[ft_force_csv_logger] run_base=%s  out_dir=%s", self.run_base, self.out_dir)

    # arm/disarm
    def _on_log_start(self, _):
        if self.recording: return
        self.traj_idx += 1
        stem = f"{self.run_base}_t{self.traj_idx:03d}_ft"
        self.csv_path = os.path.join(self.out_dir, stem + ".csv")
        self.csv_f = open(self.csv_path, 'w', newline='')
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow(['ros_time_sec','fx','fy','fz'])  # transformed forces only
        self.recording = True
        self.last_flush = rospy.Time.now().to_sec()
        rospy.loginfo("[ft_force_csv_logger] START -> %s", self.csv_path)

    def _on_log_stop(self, _):
        if not self.recording: return
        self.recording = False
        if self.csv_f:
            self.csv_f.flush(); self.csv_f.close()
            self.csv_f = None; self.csv_w = None
        rospy.loginfo("[ft_force_csv_logger] STOP (disarm)")

    # ingest
    def _on_ft(self, msg: WrenchStamped):
        if not self.recording or self.csv_w is None: return
        fx_t = msg.wrench.force.x
        fy_t = msg.wrench.force.y
        fz_t = msg.wrench.force.z
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        with self.csv_lock:
            self.csv_w.writerow([f"{t:.6f}", f"{fx_t:.9f}", f"{fy_t:.9f}", f"{fz_t:.9f}"])
            now = rospy.Time.now().to_sec()
            if (now - self.last_flush) >= FLUSH_PERIOD:
                self.csv_f.flush()
                self.last_flush = now

    def shutdown(self):
        self._on_log_stop(None)

if __name__ == '__main__':
    rospy.init_node('ft_force_csv_logger')
    node = FTForceCSVLogger()
    rospy.on_shutdown(getattr(node, 'shutdown', lambda: None))
    rospy.spin()
