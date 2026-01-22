#!/usr/bin/env python3
import os, csv, cv2, rospy, rospkg
from threading import Lock
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped, PoseStamped
from collections import deque
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Empty, Bool
from datetime import datetime
from tf.transformations import euler_from_quaternion

# For KDL TF lookups since TF is broken...
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL as kdl

WARMUP_FRAMES_DEFAULT = 10
FPS_HINT_DEFAULT = 30

class FTClockLogger:
    """
    This node logs synchronized force-torque (FT), EE pose, and AprilTag detection data
    to a CSV file, triggered by FT ssampling clock (~ 500 Hz). Also record video
    overlays of the detections to an MP4 file.

    IT CURRENTLY USES THE HIGHEST_FREQUENCY DATASTREAM AS THE CLOCK (FT SENSOR)
    """
    def __init__(self):
        self.run_base   = rospy.get_param('~run_base', None)
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.ee_frame   = rospy.get_param('~ee_frame',   'finger_tip')
        # --- params for topics ---
        self.ft_stream_topic    = rospy.get_param('~ft_topic',   '/netft_data_transformed')
        self.tag_topic          = rospy.get_param('~dets_topic_name', '/tag_detections')
        self.image_topic        = rospy.get_param('~image_topic', '/tag_detections_image')  # overlay from apriltag_ros
        # --- params for tag detections + video ---
        # self.tag_max_age    = float(rospy.get_param('~tag_max_age', 0.10))
        self.flush_period   = float(rospy.get_param('~flush_period', 0.25))
        self.fps_hint       = float(rospy.get_param('~fps_hint', FPS_HINT_DEFAULT))
        # self.warmup_frames  = int(rospy.get_param('~warmup_frames', WARMUP_FRAMES_DEFAULT))
        self.last_flush     = rospy.Time.now().to_sec()
        # --- params for EE pose ---
        self.tip_pose_max_age = float(rospy.get_param('~tip_pose_max_age', 0.02)) # 20ms default
        self.tip_pose_buf_len = int(rospy.get_param('~tip_pose_buf_len', 400)) # ~4s at 100Hz
        self._tip_buf       = deque(maxlen=self.tip_pose_buf_len)  # (t, [x,y,z,qx,qy,qz,qw])

        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        pkg = rospkg.RosPack().get_path('com_3d')
        self.out_dir = os.path.join(pkg, 'experiments'); os.makedirs(self.out_dir, exist_ok=True)
        self.traj_idx = 0
        self.recording = False
        self.csv_f = self.csv_w = None
        self.csv_lock = Lock()
        self.last_flush = rospy.Time.now().to_sec()

        # --- video state ---
        self.bridge = CvBridge()
        self.writer = None
        # self.frames_seen = 0
        self.video_path = None

        # FT state (contact, triggered, etc.)
        self.ft_contact_flag = 0
        self.ft_trigger_flag = 0

        # latest tag (sample-and-hold; no interp)
        self.tag_latest = None
        self.tag_last_rpy = None

        # EE pose from FK node (uses KDL accessed by controller, too)
        self._EE_lock = Lock()
        self.ee_last = [0.0] * 7  # fallback EE pose
        

        rospy.Subscriber(self.ft_stream_topic, WrenchStamped, self._on_ft, queue_size=500)
        rospy.Subscriber(self.tag_topic, AprilTagDetectionArray, self._on_tag, queue_size=50)
        rospy.Subscriber('/com_3d/log_start', Empty, self._start, queue_size=1)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._stop,  queue_size=1)
        rospy.Subscriber('/com_3d/fw_contact_status', Bool, self._on_fw_contact)
        rospy.Subscriber('/com_3d/fw_trigger_status', Bool, self._on_fw_trigger)
        # --- image subscriber (buffered) ---
        rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=10, buff_size=2**24)
        # rospy.Subscriber("/egm/joint_states", JointState, self._on_joint_state, queue_size=100)
        # --- tip pose subscriber (buffered) ---
        rospy.Subscriber("/com_3d/tip_pose_fk", PoseStamped, self._on_tip_pose, queue_size=200)
        

    def _start(self, _):
        with self.csv_lock:
            if self.recording: return
            # self.traj_idx += 1
            # Build a base name from datetime + object name
            object_name = rospy.get_param('/com_3d/object_name', 'unknown')
            n_safety = round(rospy.get_param('/com_3d/n_safety', 0.0), 2)
            base = self.run_base if self.run_base is not None else f"{self.timestamp}_{object_name}"
            stem = f"{base}_{n_safety}" # t{self.traj_idx:02d}"

            # Publish current log stem and dir for other nodes (e.g. live plotter and estimator)
            rospy.set_param('/com_3d/current_log_stem', stem)
            rospy.set_param('/com_3d/current_log_dir',  self.out_dir)

            path = os.path.join(self.out_dir, stem + ".csv")
            self.video_path = os.path.join(self.out_dir, stem + ".mp4")
            
            self.csv_f = open(path, 'w', newline='')
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow([
                'ros_time_sec',
                'fx','fy','fz','mx','my','mz',
                'ee_x','ee_y','ee_z','ee_qx','ee_qy','ee_qz','ee_qw',
                'tag_visible','tag_id',
                'tag_qx','tag_qy','tag_qz','tag_qw',
                'tag_x','tag_y','tag_z',
                'fw_contact','ft_trigger'
            ])
            self.recording = True
            self.last_flush = rospy.Time.now().to_sec()
            # reset video state
            self.writer = None
            # self.frames_seen = 0
            self.ft_contact_flag = 0
            self.ft_trigger_flag = 0
        rospy.loginfo("[ft_clock_logger] START -> %s", path)

    def _stop(self, _):
        with self.csv_lock:
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
        # if not msg.detections: return

        # timestamp of this detection
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

        # If no detections explicitly clear latest so logging shows NaNs instead of stale data
        if not msg.detections: 
            with self.csv_lock:
                self.tag_latest = None
            return
        # Additional catch if somehow no detections but non-empty array
        elif len(msg.detections) < 1:
            self.tag_latest = None
            return
        
        # single tag: take first detection
        det = msg.detections[0]
        p = det.pose.pose.pose
        q = p.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # roll, pitch, yaw = euler_from_quaternion(q, axes='sxyz')
        tx, ty, tz = p.position.x, p.position.y, p.position.z
        tag_id = det.id[0] if det.id else -1

        # store latest (optional, not used for row-writing in tag-clock)
        with self.csv_lock:
            self.tag_latest = {'t': t, 'id': int(tag_id),
                               'tq': (qx, qy, qz, qw),
                               'txyz': (tx, ty, tz)}

    def _on_image(self, msg: Image):
        # Write overlay frames to MP4 after a short warmup
        with self.csv_lock:
            if not self.recording: return
            # Convert
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception:
                return

            # Lazy-open writer with the first frame’s size
            if self.writer is None:
                h, w = frame.shape[:2]
                self.writer = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*'mp4v'), self.fps_hint, (w, h), True)
                if not self.writer.isOpened():
                    rospy.logerr("[ft_clock_logger] Failed to open VideoWriter for %s", self.video_path)
                    # Don’t abort logging; just skip video
                    self.writer = None
                    return

            # self.frames_seen += 1
            if self.writer is not None:
                try:
                    self.writer.write(frame)
                except Exception:
                    pass


    def _on_ft(self, msg: WrenchStamped):
        """
        FT Callback: capture the current FT and retrieve the latest tag info. 
        This uses the FT sensor as the clock for synchronization.
        """
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        ft = (float(msg.wrench.force.x),  float(msg.wrench.force.y),  float(msg.wrench.force.z),
              float(msg.wrench.torque.x), float(msg.wrench.torque.y), float(msg.wrench.torque.z))

        # FT at tag time: sample-and-hold (use the most recent FT sample seen)
        if ft is None:
            return  # no FT seen yet; skip this row until first FT arrives

        ## Retrieve the latest tag and EE info
        with self.csv_lock:
            tag = self.tag_latest
        # if tag is not None and abs(tag['t'] - t) > self.tag_max_age:
        #     tag = None  # too old
        # TEMP HACK: DISABLING TAG AGE CHECKING
        
        ee  = self._get_tip_pose_at(t)

        # Write row (tag_visible=1 because we write only when a tag exists)
        with self.csv_lock:
            if not self.recording or self.csv_w is None:
                return
            
            if tag is not None:
                tq = tag['tq']
                txyz = tag['txyz']
                tag_row = (
                    1, tag['id'], 
                    f"{tq[0]:.5f}", f"{tq[1]:.5f}", f"{tq[2]:.5f}", f"{tq[3]:.5f}",
                    f"{txyz[0]:.5f}", f"{txyz[1]:.5f}", f"{txyz[2]:.5f}"
                )
            else:
                tag_row = (0, -1, "nan", "nan", "nan", "nan", "nan", "nan", "nan")
                
            self.csv_w.writerow([
                f"{t:.6f}",
                f"{ft[0]:.5f}",  f"{ft[1]:.5f}",  f"{ft[2]:.5f}",
                f"{ft[3]:.5f}",  f"{ft[4]:.5f}",  f"{ft[5]:.5f}",
                f"{ee[0]:.5f}",  f"{ee[1]:.5f}",  f"{ee[2]:.5f}",
                f"{ee[3]:.5f}",  f"{ee[4]:.5f}",  f"{ee[5]:.5f}", f"{ee[6]:.5f}",
                *tag_row,
                self.ft_contact_flag, self.ft_trigger_flag
            ])
            now = rospy.Time.now().to_sec()
            if (now - self.last_flush) >= self.flush_period:
                self.csv_f.flush(); self.last_flush = now

    
    def _on_fw_contact(self, msg: Bool):
        with self.csv_lock:
            if self.recording:
                self.ft_contact_flag = 1 if msg.data else 0

    def _on_fw_trigger(self, msg: Bool):
        with self.csv_lock:
            if self.recording:
                self.ft_trigger_flag = 1 if msg.data else 0

    def _on_tip_pose(self, msg: PoseStamped):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        p = msg.pose.position
        q = msg.pose.orientation
        ee = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        with self._EE_lock:
            self._tip_buf.append((t, ee))

    def _get_tip_pose_at(self, t_query: float):
        """
        Return tip pose sample closest to t_query, or last known pose if none fresh enough.
        """
        with self._EE_lock:
            buf = list(self._tip_buf)

        if not buf:
            return self.ee_last[:]  # fallback

        # nearest neighbor in time
        t_best, ee_best = min(buf, key=lambda x: abs(x[0] - t_query))
        if abs(t_best - t_query) > self.tip_pose_max_age:
            # too stale: use last known pose (or still use ee_best if you prefer)
            return self.ee_last[:]

        self.ee_last = ee_best[:]  # update fallback
        return ee_best

        


if __name__ == '__main__':
    rospy.init_node('ft_clock_logger')
    node = FTClockLogger()
    rospy.on_shutdown(getattr(node, 'shutdown', lambda: None))
    rospy.spin()
