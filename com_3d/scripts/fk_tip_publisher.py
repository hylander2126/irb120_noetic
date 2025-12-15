#!/usr/bin/env python3
import rospy
import math
import hashlib
from threading import Lock

import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL as kdl


class FKTipPublisher:
    """
    Subscribes to joint states, computes FK base_link -> tip_link using KDL,
    and publishes PoseStamped on /com_3d/tip_pose_fk continuously (on each joint update).

    This is intended to be the single source of truth for EE pose used by:
      - logger (for recording)
      - controller (for z-lock / orientation lock)
    """

    def __init__(self):
        self.base_link = rospy.get_param("~base_link", "base_link")
        self.tip_link  = rospy.get_param("~tip_link",  "finger_tip")
        self.urdf_param = rospy.get_param("~urdf_param", "robot_description")

        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/egm/joint_states")
        self.out_topic         = rospy.get_param("~out_topic", "/com_3d/tip_pose_fk")

        # Optional: warn if FK fails too often
        self.warn_throttle_sec = float(rospy.get_param("~warn_throttle_sec", 1.0))

        # Load URDF and build KDL chain
        urdf_xml = rospy.get_param(self.urdf_param)
        self.urdf_hash = hashlib.sha256(urdf_xml.encode("utf-8")).hexdigest()[:12]
        robot = URDF.from_xml_string(urdf_xml)

        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("[fk_tip_publisher] Failed to build KDL tree from URDF")

        self.chain = tree.getChain(self.base_link, self.tip_link)
        self.nj = self.chain.getNrOfJoints()
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

        # Extract KDL joint order (movable joints only)
        self.kdl_joint_names = []
        for i in range(self.chain.getNrOfSegments()):
            seg = self.chain.getSegment(i)
            jnt = seg.getJoint()
            if jnt.getTypeName() != "None":
                self.kdl_joint_names.append(jnt.getName())

        if len(self.kdl_joint_names) != self.nj:
            rospy.logwarn(
                "[fk_tip_publisher] KDL joint list length (%d) != nj (%d). Names: %s",
                len(self.kdl_joint_names), self.nj, self.kdl_joint_names
            )

        rospy.loginfo(
            "[fk_tip_publisher] FK chain %s -> %s | nj=%d | urdf_hash=%s",
            self.base_link, self.tip_link, self.nj, self.urdf_hash
        )
        rospy.loginfo("[fk_tip_publisher] Joint order: %s", self.kdl_joint_names)

        self._lock = Lock()
        self._last_pose = None  # PoseStamped

        self.pub = rospy.Publisher(self.out_topic, PoseStamped, queue_size=200)
        self.sub = rospy.Subscriber(self.joint_state_topic, JointState, self._on_joint, queue_size=200)

    def _on_joint(self, msg: JointState):
        # Map joint state into KDL order
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        q = kdl.JntArray(self.nj)

        for i, jn in enumerate(self.kdl_joint_names):
            if jn not in name_to_idx:
                # Incomplete joint state, skip
                rospy.logwarn_throttle(
                    self.warn_throttle_sec,
                    "[fk_tip_publisher] JointState missing '%s' (have %d names). Skipping FK.",
                    jn, len(msg.name)
                )
                return
            q[i] = float(msg.position[name_to_idx[jn]])

        # Compute FK
        try:
            F = kdl.Frame()
            self.fk_solver.JntToCart(q, F)

            p = F.p
            qx, qy, qz, qw = F.M.GetQuaternion()  # (x,y,z,w)

            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
            ps.header.frame_id = self.base_link

            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = float(p[2])

            ps.pose.orientation.x = float(qx)
            ps.pose.orientation.y = float(qy)
            ps.pose.orientation.z = float(qz)
            ps.pose.orientation.w = float(qw)

            with self._lock:
                self._last_pose = ps

            self.pub.publish(ps)

        except Exception as e:
            rospy.logwarn_throttle(
                self.warn_throttle_sec,
                "[fk_tip_publisher] FK failed: %s",
                str(e)
            )

    def get_last_pose(self):
        with self._lock:
            return self._last_pose


if __name__ == "__main__":
    rospy.init_node("fk_tip_publisher")
    node = FKTipPublisher()
    rospy.spin()
