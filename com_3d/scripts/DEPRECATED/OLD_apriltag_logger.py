#!/usr/bin/env python3
import os, csv, cv2, rospy, rospkg
from threading import Lock
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from tf.transformations import euler_from_quaternion

WARMUP_FRAMES = 30
FPS_HINT = 30
FOURCC = cv2.VideoWriter_fourcc(*'mp4v')  # try 'XVID' if mp4 issues

class AprilTagVideoLoggerFixedNS:
    def __init__(self):
        # --- required shared name from launch ---
        self.run_base = rospy.get_param('~run_base', None)
        if not self.run_base:
            rospy.logfatal("[apriltag_video_logger] Missing required param ~run_base")
            rospy.signal_shutdown("missing ~run_base")
            return

        # --- detector namespace + topics ---
        # detector_ns = rospy.get_param('~detector_ns', 'apriltag_ros_continuous_node').strip('/')
        detector_ns = ''
        self.dets_topic  = detector_ns + '/' + rospy.get_param('~dets_topic_name', 'tag_detections').lstrip('/')
        # camera image topic (for recording video)
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')

        # --- outputs under com_3d/experiments ---
        pkg_path = rospkg.RosPack().get_path('com_3d')
        out_dir  = os.path.join(pkg_path, 'experiments')
        os.makedirs(out_dir, exist_ok=True)
        self.video_path = os.path.join(out_dir, f"{self.run_base}_tag.mp4")
        self.csv_path   = os.path.join(out_dir, f"{self.run_base}_tag.csv")

        # state
        self.traj_idx = 0 # Used to not overwrite successive runs 
        self.bridge = CvBridge()
        self.writer = None
        self.frame_size = None
        self.frames_seen = 0
        self.recording = False

        # I/O + synchronization
        self.io_lock = Lock()
        self.csv_f = open(self.csv_path, 'w', newline='')
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow(['ros_time_sec','tag_id','roll(rad)','pitch(rad)','yaw(rad)',
                             'tx(m)','ty(m)','tz(m)','q_x','q_y','q_z','q_w'])

        # subs
        rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=10, buff_size=2**24)
        rospy.Subscriber(self.dets_topic, AprilTagDetectionArray, self._on_dets, queue_size=10)
        rospy.Subscriber('/egm/joint_trajectory_controller/follow_joint_trajectory/goal',
                         FollowJointTrajectoryActionGoal, self._on_goal, queue_size=1)
        rospy.Subscriber('/egm/joint_trajectory_controller/follow_joint_trajectory/result',
                         FollowJointTrajectoryActionResult, self._on_result, queue_size=1)

        rospy.loginfo("[apriltag_video_logger] run_base=%s", self.run_base)
        rospy.loginfo("  subscribing dets: %s", self.dets_topic)
        rospy.loginfo("  subscribing image: %s", self.image_topic)
        rospy.loginfo("  writing video: %s", self.video_path)
        rospy.loginfo("  writing csv  : %s", self.csv_path)

    # lifecycle
    def _start(self):
        with self.io_lock:
            if self.recording:
                return
            self.traj_idx += 1
            self.recording = True
            self.frames_seen = 0
            rospy.loginfo("[apriltag_video_logger] START")

    def _stop(self, reason="result"):
        with self.io_lock:
            if not self.recording:
                return
            self.recording = False
            rospy.loginfo("[apriltag_video_logger] STOP (%s)", reason)
            if self.writer is not None:
                try:
                    self.writer.release()
                except Exception as e:
                    rospy.logwarn("VideoWriter.release error: %s", e)
                self.writer = None
            if self.csv_f:
                self.csv_f.flush()

    # callbacks
    def _on_goal(self, _msg):
        self._start()

    def _on_result(self, _msg):
        self._stop("trajectory_result")

    def _on_image(self, msg: Image):
        with self.io_lock:
            if not self.recording:
                return
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.writer is None:
                h, w = cv_img.shape[:2]
                self.frame_size = (w, h)
                self.writer = cv2.VideoWriter(self.video_path, FOURCC, float(FPS_HINT), (w, h), True)
                if not self.writer.isOpened():
                    rospy.logerr("[apriltag_video_logger] Failed to open VideoWriter")
                    self._stop("videowriter_error")
                    return
                rospy.loginfo("[apriltag_video_logger] VideoWriter opened %dx%d @ %dfps", w, h, FPS_HINT)

            self.frames_seen += 1
            if self.frames_seen > WARMUP_FRAMES:
                try:
                    self.writer.write(cv_img)
                except Exception as e:
                    rospy.logwarn("VideoWriter.write error: %s", e)

    def _on_dets(self, msg: AprilTagDetectionArray):
        if not self.recording:
            return
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        for det in msg.detections:
            p = det.pose.pose.pose
            q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            r, pitch, yaw = euler_from_quaternion(q, axes='sxyz')
            tx, ty, tz = p.position.x, p.position.y, p.position.z
            tag_id = det.id[0] if det.id else -1
            with self.io_lock:
                self.csv_w.writerow([
                    f"{t:.6f}", tag_id,
                    f"{r:.6f}", f"{pitch:.6f}", f"{yaw:.6f}",
                    f"{tx:.6f}", f"{ty:.6f}", f"{tz:.6f}",
                    f"{q[0]:.6f}", f"{q[1]:.6f}", f"{q[2]:.6f}", f"{q[3]:.6f}"
                ])

    def shutdown(self):
        self._stop("shutdown")
        with self.io_lock:
            if self.csv_f:
                try:
                    self.csv_f.close()
                except Exception:
                    pass
                self.csv_f = None

if __name__ == '__main__':
    # Reduce OpenCV thread contention (sometimes helps with shutdown races)
    try:
        cv2.setNumThreads(1)
    except Exception:
        pass

    rospy.init_node('apriltag_video_logger')
    node = AprilTagVideoLoggerFixedNS()
    rospy.on_shutdown(getattr(node, 'shutdown', lambda: None))
    rospy.spin()
