#!/usr/bin/env python3
import os, csv, cv2, rospy, rospkg
from threading import Lock
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

WARMUP_FRAMES = 10
FPS_HINT = 30
FOURCC = cv2.VideoWriter_fourcc(*'mp4v')

class AprilTagVideoLogger:
    def __init__(self):
        self.run_base = rospy.get_param('~run_base', None)
        if not self.run_base:
            rospy.logfatal("[apriltag_video_logger] Missing ~run_base"); rospy.signal_shutdown("missing ~run_base"); return

        # detector_ns = rospy.get_param('~detector_ns', 'apriltag_ros_continuous_node').strip('/')
        self.dets_topic  = '/' + rospy.get_param('~dets_topic_name', 'tag_detections').lstrip('/')
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')

        pkg_path = rospkg.RosPack().get_path('com_3d')
        self.out_dir  = os.path.join(pkg_path, 'experiments'); os.makedirs(self.out_dir, exist_ok=True)

        self.traj_idx = 0
        self.recording = False
        self.bridge = CvBridge()
        self.io_lock = Lock()
        self.writer = None
        self.frames_seen = 0
        self.csv_f = None
        self.csv_w = None
        self.video_path = None
        self.csv_path = None

        rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=10, buff_size=2**24)
        rospy.Subscriber(self.dets_topic,  AprilTagDetectionArray, self._on_dets, queue_size=10)

        # Arm/disarm control
        rospy.Subscriber('/com_3d/log_start', Empty, self._on_log_start, queue_size=1)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._on_log_stop,  queue_size=1)

        rospy.loginfo("[apriltag_video_logger] run_base=%s  dets=%s  img=%s", self.run_base, self.dets_topic, self.image_topic)

    # ---- arm/disarm -> start/stop ----
    def _on_log_start(self, _):
        with self.io_lock:
            if self.recording: return
            self.traj_idx += 1
            stem = f"{self.run_base}_t{self.traj_idx:03d}_tag"
            self.video_path = os.path.join(self.out_dir, stem + ".mp4")
            self.csv_path   = os.path.join(self.out_dir, stem + ".csv")
            # new CSV for this arm
            self.csv_f = open(self.csv_path, 'w', newline='')
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow(['ros_time_sec','tag_id','roll(rad)','pitch(rad)','yaw(rad)',
                                 'tx(m)','ty(m)','tz(m)','q_x','q_y','q_z','q_w'])
            self.writer = None
            self.frames_seen = 0
            self.recording = True
            rospy.loginfo("[apriltag_video_logger] START -> %s / %s", self.video_path, self.csv_path)

    def _on_log_stop(self, _):
        with self.io_lock:
            if not self.recording: return
            self.recording = False
            rospy.loginfo("[apriltag_video_logger] STOP (disarm)")
            try:
                if self.writer is not None:
                    self.writer.release()
            except Exception: pass
            self.writer = None
            if self.csv_f:
                self.csv_f.flush(); self.csv_f.close()
                self.csv_f = None; self.csv_w = None

    # ---- data callbacks ----
    def _on_image(self, msg: Image):
        with self.io_lock:
            if not self.recording: return
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.writer is None:
                h, w = cv_img.shape[:2]
                self.writer = cv2.VideoWriter(self.video_path, FOURCC, float(FPS_HINT), (w, h), True)
                if not self.writer.isOpened():
                    rospy.logerr("[apriltag_video_logger] Failed to open VideoWriter")
                    self._on_log_stop(None); return
                rospy.loginfo("[apriltag_video_logger] VideoWriter opened %dx%d @ %dfps", w, h, FPS_HINT)
            self.frames_seen += 1
            if self.frames_seen > WARMUP_FRAMES:
                try: self.writer.write(cv_img)
                except Exception: pass

    def _on_dets(self, msg: AprilTagDetectionArray):
        if not self.recording or self.csv_w is None: return
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
        self._on_log_stop(None)

if __name__ == '__main__':
    try: cv2.setNumThreads(1)
    except Exception: pass
    rospy.init_node('apriltag_video_logger')
    node = AprilTagVideoLogger()
    rospy.on_shutdown(getattr(node, 'shutdown', lambda: None))
    rospy.spin()
