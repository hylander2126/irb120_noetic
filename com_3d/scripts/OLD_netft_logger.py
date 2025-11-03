#!/usr/bin/env python3
import os, csv, rospy, rospkg
from threading import Lock
from geometry_msgs.msg import WrenchStamped
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult

FLUSH_PERIOD = 0.25  # seconds

class FTForceCSVLogger:
    def __init__(self):
        # Topics
        self.ft_topic     = '/netft_data'
        self.goal_topic   = '/egm/joint_trajectory_controller/follow_joint_trajectory/goal'
        self.result_topic = '/egm/joint_trajectory_controller/follow_joint_trajectory/result'

        # Required: run_base from launch param
        self.run_base = rospy.get_param('~run_base', None)
        if not self.run_base:
            rospy.logfatal("[ft_force_csv_logger] Missing required param ~run_base")
            rospy.signal_shutdown("missing ~run_base")
            return

        # Output path under com_3d/experiments
        pkg_path = rospkg.RosPack().get_path('com_3d')
        self.out_dir = os.path.join(pkg_path, 'experiments')
        os.makedirs(self.out_dir, exist_ok=True)

        self.csv_path = os.path.join(self.out_dir, f"{self.run_base}_ft.csv")
        self.csv_f = None
        self.csv_w = None
        self.csv_lock = Lock()

        self.recording = False
        self.last_flush = rospy.Time.now().to_sec()

        rospy.Subscriber(self.ft_topic, WrenchStamped, self._on_ft, queue_size=50)
        rospy.Subscriber(self.goal_topic,   FollowJointTrajectoryActionGoal,   self._on_goal,   queue_size=1)
        rospy.Subscriber(self.result_topic, FollowJointTrajectoryActionResult, self._on_result, queue_size=1)

        rospy.loginfo("[ft_force_csv_logger] Ready. run_base=%s -> %s", self.run_base, self.csv_path)

    def _open_csv(self):
        if self.csv_f: return
        self.csv_f = open(self.csv_path, 'w', newline='')
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow(['ros_time_sec','fx','fy','fz'])  # transformed (fixed flip)

    def _close_csv(self):
        if self.csv_f:
            self.csv_f.flush()
            self.csv_f.close()
            self.csv_f = None
            self.csv_w = None

    def _on_goal(self, _msg):
        if self.recording: return
        self._open_csv()
        self.recording = True
        self.last_flush = rospy.Time.now().to_sec()
        rospy.loginfo("[ft_force_csv_logger] START")

    def _on_result(self, _msg):
        if not self.recording: return
        self.recording = False
        self._close_csv()
        rospy.loginfo("[ft_force_csv_logger] STOP (trajectory result)")

    def _on_ft(self, msg: WrenchStamped):
        if not self.recording or self.csv_w is None: return
        # Fixed transform: (-pi about +y) then (-pi about +x) => diag([-1,-1,+1])
        fx_t = -msg.wrench.force.x
        fy_t = -msg.wrench.force.y
        fz_t =  msg.wrench.force.z
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        with self.csv_lock:
            self.csv_w.writerow([f"{t:.6f}", f"{fx_t:.9f}", f"{fy_t:.9f}", f"{fz_t:.9f}"])
            now = rospy.Time.now().to_sec()
            if (now - self.last_flush) >= FLUSH_PERIOD:
                self.csv_f.flush()
                self.last_flush = now

    def shutdown(self):
        self._close_csv()

if __name__ == '__main__':
    rospy.init_node('ft_force_csv_logger')
    node = FTForceCSVLogger()
    rospy.on_shutdown(getattr(node, 'shutdown', lambda: None))
    rospy.spin()
