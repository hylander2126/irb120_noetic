#!/usr/bin/env python3
import os, csv, cv2, rospy, rospkg
from threading import Lock
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped, TransformStamped
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
        self.tag_max_age    = float(rospy.get_param('~tag_max_age', 0.10))
        self.flush_period   = float(rospy.get_param('~flush_period', 0.25))
        self.fps_hint       = float(rospy.get_param('~fps_hint', FPS_HINT_DEFAULT))
        # self.warmup_frames  = int(rospy.get_param('~warmup_frames', WARMUP_FRAMES_DEFAULT))
        self.last_flush     = rospy.Time.now().to_sec()

        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

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
        # self.frames_seen = 0
        self.video_path = None

        # FT state (contact, triggered, etc.)
        self.ft_contact_flag = 0
        self.ft_trigger_flag = 0

        # latest tag (sample-and-hold; no interp)
        self.tag_latest = {'t': None, 'id': None,
                               'rpy': (None, None, None),
                               'txyz': (None, None, None)}

        # --- KDL FK for EE pose (base_link -> finger_tip) ---
        robot = URDF.from_parameter_server('robot_description')
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            rospy.logerr("[ft_clock_logger] Failed to build KDL tree from URDF.")
            raise RuntimeError("KDL tree error")

        self.chain = tree.getChain(self.base_frame, self.ee_frame)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.nj = self.chain.getNrOfJoints()

        # Joint names in KDL order (only movable joints)
        self.kdl_joint_names = []
        for i in range(self.chain.getNrOfSegments()):
            seg = self.chain.getSegment(i)
            jnt = seg.getJoint()
            # In PyKDL Python, easiest check:
            if jnt.getTypeName() != "None":
                self.kdl_joint_names.append(jnt.getName())

        self._q_lock = Lock()
        self._q = None  # latest joint positions in KDL order
        self.ee_last = [0.0] * 7  # fallback EE pose
        

        rospy.Subscriber(self.ft_stream_topic, WrenchStamped, self._on_ft, queue_size=500)
        rospy.Subscriber(self.tag_topic, AprilTagDetectionArray, self._on_tag, queue_size=50)
        rospy.Subscriber('/com_3d/log_start', Empty, self._start, queue_size=1)
        rospy.Subscriber('/com_3d/log_stop',  Empty, self._stop,  queue_size=1)
        rospy.Subscriber('/com_3d/fw_contact_status', Bool, self._on_fw_contact)
        rospy.Subscriber('/com_3d/fw_trigger_status', Bool, self._on_fw_trigger)
        # --- image subscriber (buffered) ---
        rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=10, buff_size=2**24)
        rospy.Subscriber("/egm/joint_states", JointState, self._on_joint_state, queue_size=100)
        

    def _start(self, _):
        with self.lock:
            if self.recording: return
            self.traj_idx += 1
            # Build a base name from datetime + object name
            object_name = rospy.get_param('/com_3d/object_name', 'unknown')
            base = self.run_base if self.run_base is not None else f"{self.timestamp}_{object_name}"
            stem = f"{base}_t{self.traj_idx:02d}"

            # stem = f"{self.run_base}_t{self.traj_idx:03d}_SYNC"
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

        # timestamp of this detection
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

        # single tag: take first detection
        det = msg.detections[0]
        p = det.pose.pose.pose
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q, axes='sxyz')
        tx, ty, tz = p.position.x, p.position.y, p.position.z
        tag_id = det.id[0] if det.id else -1

        # store latest (optional, not used for row-writing in tag-clock)
        with self.lock:
            self.tag_latest = {'t': t, 'id': int(tag_id),
                               'rpy': (roll, pitch, yaw),
                               'txyz': (tx, ty, tz)}

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
                self.writer = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*'mp4v'), self.fps_hint, (w, h), True)
                if not self.writer.isOpened():
                    rospy.logerr("[ft_clock_logger] Failed to open VideoWriter for %s", self.video_path)
                    # Don’t abort logging; just skip video
                    self.writer = None
                    return

            # self.frames_seen += 1
            # if self.frames_seen > self.warmup_frames and self.writer is not None:
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
        

        ## Retrieve the latest tag info
        with self.lock:
            tag = self.tag_latest

        # only write when armed
        with self.lock:
            if not self.recording or self.csv_w is None:
                return

        # EE pose at tag time; on failure, hold last good
        # ee = [float('nan')]*7
        # try:
        #     ts: TransformStamped = self.tfbuf.lookup_transform(
        #         self.base_frame, self.ee_frame,
        #         rospy.Time.from_sec(t), rospy.Duration(self.tf_timeout_sec)
        #     )
        #     ee = [
        #         ts.transform.translation.x,
        #         ts.transform.translation.y,
        #         ts.transform.translation.z,
        #         ts.transform.rotation.x,
        #         ts.transform.rotation.y,
        #         ts.transform.rotation.z,
        #         ts.transform.rotation.w,
        #     ]
        #     self.ee_last = ee[:]
        # except Exception as e:
        #     rospy.logwarn(f"Failed to lookup transform from {self.base_frame} to {self.ee_frame} at time {t}: {e}")
        #     if self.ee_last is not None:
        #         ee = self.ee_last[:]
        ee = self._get_latest_ee()


        # Write row (tag_visible=1 because we write only when a tag exists)
        with self.lock:
            if not self.recording or self.csv_w is None:
                return
            self.csv_w.writerow([
                f"{t:.6f}",
                f"{ft[0]:.9f}",  f"{ft[1]:.9f}",  f"{ft[2]:.9f}",
                f"{ft[3]:.9f}",  f"{ft[4]:.9f}",  f"{ft[5]:.9f}",
                f"{ee[0]:.9f}", f"{ee[1]:.9f}", f"{ee[2]:.9f}",
                f"{ee[3]:.9f}", f"{ee[4]:.9f}", f"{ee[5]:.9f}", f"{ee[6]:.9f}",
                1, int(tag['id']),
                f"{tag['rpy'][0]:.9f}", f"{tag['rpy'][1]:.9f}", f"{tag['rpy'][2]:.9f}",
                f"{tag['txyz'][0]:.9f}",   f"{tag['txyz'][1]:.9f}",    f"{tag['txyz'][2]:.9f}",
                self.ft_contact_flag, self.ft_trigger_flag
            ])
            now = rospy.Time.now().to_sec()
            if (now - self.last_flush) >= self.flush_period:
                self.csv_f.flush(); self.last_flush = now

    
    def _on_fw_contact(self, msg: Bool):
        with self.lock:
            if msg.data:
                self.ft_contact_flag = 1

    def _on_fw_trigger(self, msg: Bool):
        with self.lock:
            if msg.data:
                self.ft_trigger_flag = 1

    def _on_joint_state(self, msg: JointState):
        """Maintain latest joint vector in KDL joint order."""
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}

        q = []
        for jn in self.kdl_joint_names:
            if jn not in name_to_pos:
                # Incomplete; skip this message
                return
            q.append(name_to_pos[jn])

        with self._q_lock:
            self._q = q

    def _get_latest_ee(self):
        """EE pose via KDL FK using latest joint state"""
        with self._q_lock:
            q_curr = None if self._q is None else list(self._q)

        if q_curr is not None and len(q_curr) == self.nj:
            try:
                q_kdl = kdl.JntArray(self.nj)
                for i, v in enumerate(q_curr):
                    q_kdl[i] = v

                F = kdl.Frame()
                self.fk_solver.JntToCart(q_kdl, F)

                p = F.p
                qx, qy, qz, qw = F.M.GetQuaternion()

                ee = [p[0], p[1], p[2], qx, qy, qz, qw]
                self.ee_last = ee[:]
            except Exception as e:
                rospy.logwarn_throttle(
                    1.0,
                    f"[ft_clock_logger] FK failed, using last EE pose: {e}"
                )
                ee = self.ee_last[:]
        else:
            # No valid joint state yet; use last known EE pose
            ee = self.ee_last[:]

        return ee

        


if __name__ == '__main__':
    rospy.init_node('ft_clock_logger')
    node = FTClockLogger()
    rospy.on_shutdown(getattr(node, 'shutdown', lambda: None))
    rospy.spin()
