#!/usr/bin/env python3
import os, csv, cv2, rospy, rospkg
from threading import Lock
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped, TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Empty
import tf2_ros
from tf.transformations import euler_from_quaternion

WARMUP_FRAMES_DEFAULT = 10
FPS_HINT_DEFAULT = 30
FOURCC = cv2.VideoWriter_fourcc(*'mp4v')

FT_BUFFER_SEC = 2.0 # Keep this much FT history
MAX_DT_FT = 0.02    # require and ft sample within +-20ms of tag time

class FTClockLogger:
    def __init__(self):
        self.run_base   = rospy.get_param('~run_base')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.ee_frame   = rospy.get_param('~ee_frame',   'finger_tip')
        self.ft_topic   = rospy.get_param('~ft_topic',   '/netft_data_transformed')
        self.tag_topic  = '/' + rospy.get_param('~dets_topic_name', 'tag_detections').lstrip('/')
        self.tag_max_age = float(rospy.get_param('~tag_max_age', 0.10))
        self.flush_period = float(rospy.get_param('~flush_period', 0.25))
        # --- params for video ---
        self.image_topic   = rospy.get_param('~image_topic', '/tag_detections_image')  # overlay from apriltag_ros
        self.fps_hint      = float(rospy.get_param('~fps_hint', FPS_HINT_DEFAULT))
        self.warmup_frames = int(rospy.get_param('~warmup_frames', WARMUP_FRAMES_DEFAULT))


        pkg = rospkg.RosPack().get_path('com_3d')
        self.out_dir = os.path.join(pkg, 'experiments'); os.makedirs(self.out_dir, exist_ok=True)
        self.traj_idx = 0
        self.recording = False
        self.csv_f = self.csv_w = None
        self.lock = Lock()
        self.last_flush = rospy.Time.now().to_sec()

        # --- video state ---
        self.bridge = CvBridge()
        self.writer = None
        self.frames_seen = 0
        self.video_path = None

        # latest tag (sample-and-hold; no interp)
        self.tag_latest = None  # dict: t, id, rpy, txyz, qxyzw
        self.ft_latest = None

        # TF buffer (TF handles any needed interp internally)
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf)
        self.tf_timeout_sec = float(rospy.get_param('~tf_timeout_sec', 0.03)) # 30 ms
        self.ee_last = None
        

        rospy.Subscriber(self.tag_topic, AprilTagDetectionArray, self._on_tag, queue_size=50)
        rospy.Subscriber(self.ft_topic,  WrenchStamped,          self._on_ft,  queue_size=500)
        rospy.Subscriber('/com_3d/log_start', Empty, self._start, queue_size=1)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._stop,  queue_size=1)
        # --- image subscriber (buffered) ---
        self.img_sub = rospy.Subscriber(self.image_topic, Image, self._on_image, 
                                        queue_size=10, buff_size=2**24)

    def _start(self, _):
        with self.lock:
            if self.recording: return
            self.traj_idx += 1
            stem = f"{self.run_base}_t{self.traj_idx:03d}_SYNC"
            path = os.path.join(self.out_dir, stem + ".csv")
            self.video_path = os.path.join(self.out_dir, stem + ".mp4")
            self.csv_f = open(path, 'w', newline='')
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow([
                'ros_time_sec',
                'fx','fy','fz','mx','my','mz',
                'ee_x','ee_y','ee_z','ee_qx','ee_qy','ee_qz','ee_qw',
                'tag_visible','tag_id',
                'tag_roll(rad)','tag_pitch(rad)','tag_yaw(rad)',
                'tag_tx(m)','tag_ty(m)','tag_tz(m)',
                'tag_qx','tag_qy','tag_qz','tag_qw'
            ])
            self.recording = True
            self.last_flush = rospy.Time.now().to_sec()
            # reset video state
            self.writer = None
            self.frames_seen = 0
        rospy.loginfo("[ft_clock_logger] START -> %s", path)

    def _stop(self, _):
        with self.lock:
            if not self.recording: return
            self.recording = False
            # stop video first
            try:
                if self.writer is not None:
                    self.writer.release()
            except Exception:
                pass
            self.writer = None
            # close CSV
            try:
                if self.csv_f:
                    self.csv_f.flush()
                    self.csv_f.close()
            finally:
                self.csv_f = None
                self.csv_w = None
        rospy.loginfo("[ft_clock_logger] STOP")

    def _on_tag(self, msg: AprilTagDetectionArray):
        if not msg.detections: return
        # Use the timestamp from message header if available
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

        # Since we have 1 tag, just grab the first detection
        det = msg.detections[0]
        p = det.pose.pose.pose
        # qx,qy,qz,qw = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q, axes='sxyz')
        tx, ty, tz = p.position.x, p.position.y, p.position.z
        tag_id = det.id[0] if det.id else -1

        # Thread-safe update of latest tag
        with self.lock:
            self.tag_latest = {
                't': t,
                'id': int(tag_id),
                'rpy': (roll, pitch, yaw),
                'txyz': (tx, ty, tz),
                'qxyzw': tuple(q)
            }

    def _on_image(self, msg: Image):
        # Write overlay frames to MP4 after a short warmup
        with self.lock:
            if not self.recording: return
            # Convert
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception:
                return

            # Lazy-open writer with the first frame’s size
            if self.writer is None:
                h, w = frame.shape[:2]
                self.writer = cv2.VideoWriter(self.video_path, FOURCC, self.fps_hint, (w, h), True)
                if not self.writer.isOpened():
                    rospy.logerr("[ft_clock_logger] Failed to open VideoWriter for %s", self.video_path)
                    # Don’t abort logging; just skip video
                    self.writer = None
                    return

            self.frames_seen += 1
            if self.frames_seen > self.warmup_frames and self.writer is not None:
                try:
                    self.writer.write(frame)
                except Exception:
                    pass


    def _on_ft(self, msg: WrenchStamped):
        # fast escape to avoid work when stopped
        with self.lock:
            recording_now = self.recording
            csv_w_ref = self.csv_w
        if not recording_now or csv_w_ref is None: return
        
        # --- compute outside the lock ---
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

        # --- EE at FT time; on failure, hold last good ---
        ee = [float('nan')]*7  # will be overwritten
        try:
            ts = self.tfbuf.lookup_transform(
                self.base_frame, self.ee_frame,
                rospy.Time.from_sec(t), rospy.Duration(self.tf_timeout_sec)
            )
            ee = [
                ts.transform.translation.x,
                ts.transform.translation.y,
                ts.transform.translation.z,
                ts.transform.rotation.x,
                ts.transform.rotation.y,
                ts.transform.rotation.z,
                ts.transform.rotation.w,
            ]
            self.ee_last = ee[:]  # update last good
        except Exception:
            if self.ee_last is not None:
                ee = self.ee_last[:]  # just hold previous
            # else keep NaNs for the very first few samples before TF exists

        # Tag: latest within age (sample-and-hold)
        tag_visible, tag_id = 0, -1
        tag_rpy = (float('nan'),)*3; tag_txyz = (float('nan'),)*3; tag_q = (float('nan'),)*4
        tl = None
        with self.lock:
            tl = self.tag_latest.copy() if self.tag_latest else None
        if tl and (t - tl['t']) <= self.tag_max_age:
            tag_visible = 1
            tag_id = tl['id']
            tag_rpy = tl['rpy']
            tag_txyz = tl['txyz']
            tag_q = tl['qxyzw']

        # --- write within the lock, with a FINAL re-check ---
        with self.lock:
            if not self.recording or self.csv_w is None: return
            self.csv_w.writerow([
                f"{t:.6f}",
                f"{msg.wrench.force.x:.9f}",  f"{msg.wrench.force.y:.9f}",  f"{msg.wrench.force.z:.9f}",
                f"{msg.wrench.torque.x:.9f}", f"{msg.wrench.torque.y:.9f}", f"{msg.wrench.torque.z:.9f}",
                f"{ee[0]:.9f}", f"{ee[1]:.9f}", f"{ee[2]:.9f}",
                f"{ee[3]:.9f}", f"{ee[4]:.9f}", f"{ee[5]:.9f}", f"{ee[6]:.9f}",
                int(tag_visible), int(tag_id),
                f"{tag_rpy[0]:.9f}", f"{tag_rpy[1]:.9f}", f"{tag_rpy[2]:.9f}",
                f"{tag_txyz[0]:.9f}", f"{tag_txyz[1]:.9f}", f"{tag_txyz[2]:.9f}",
                f"{tag_q[0]:.9f}", f"{tag_q[1]:.9f}", f"{tag_q[2]:.9f}", f"{tag_q[3]:.9f}",
            ])
            now = rospy.Time.now().to_sec()
            if (now - self.last_flush) >= self.flush_period:
                self.csv_f.flush(); self.last_flush = now

if __name__ == '__main__':
    rospy.init_node('ft_clock_logger')
    node = FTClockLogger()
    rospy.on_shutdown(getattr(node, 'shutdown', lambda: None))
    rospy.spin()
