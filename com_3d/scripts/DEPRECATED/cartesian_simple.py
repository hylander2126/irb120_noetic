#!/usr/bin/env python3
from click import group
import rospy
import moveit_commander
import threading
from geometry_msgs.msg import Pose, WrenchStamped
from std_msgs.msg import Empty
from abb_robot_msgs.srv import TriggerWithResultCode
import actionlib
from moveit_msgs.msg import MoveGroupAction, Constraints, OrientationConstraint
import numpy as np

MOVE_GROUP_NAME = "manipulator"
EGM_START_SRV   = "/rws/sm_addin/start_egm_joint"
EGM_STOP_SRV    = "/rws/sm_addin/stop_egm"
POSE_REF_FRAME  = "base_link"

# ----------------------- EGM & logging -----------------------
def _start_egm():
    rospy.wait_for_service(EGM_START_SRV)
    try:
        rospy.ServiceProxy(EGM_START_SRV, TriggerWithResultCode)(TriggerWithResultCode._request_class())
    except Exception as e:
        rospy.logwarn(f"start_egm call failed: {e}")

def _stop_egm():
    rospy.wait_for_service(EGM_STOP_SRV)
    try:
        rospy.ServiceProxy(EGM_STOP_SRV, TriggerWithResultCode)(TriggerWithResultCode._request_class())
    except Exception as e:
        rospy.logwarn(f"stop_egm call failed: {e}")

def move_cartesian_simple(
    pos_goal,                 # [x, y, z] in base_frame
    quat_goal,                # [x, y, z, w] quaternion in base_frame
    duration=3.0,
    goal_time_tolerance=2.0,
    pos_tolerance=0.005,      # [m]
    rot_tolerance=0.02,       # [rad]
    base_frame="base_link",
    ee_frame="tool0",
    move_group_name="manipulator",
    check_timeout=3.0         # extra seconds for pose check
):
    """
    Barebones: given a Cartesian pose, plan once with MoveIt to get a joint goal,
    send via EGM FollowJointTrajectory, and verify final EE pose with TF.
    """
    import rospy
    import math
    import numpy as np
    from geometry_msgs.msg import Pose
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from control_msgs.msg import FollowJointTrajectoryActionGoal
    from moveit_commander import MoveGroupCommander
    import tf2_ros
    import tf.transformations as tft
    from rospy.exceptions import ROSException

    JOINT_CMD_TOPIC = "/egm/joint_trajectory_controller/follow_joint_trajectory/goal"
    JOINT_NAMES = [
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6",
    ]

    # ----------------- 1) PLAN JOINTS FROM CARTESIAN POSE (MoveIt) -----------------
    group = MoveGroupCommander(move_group_name)

    # 1. Update pos_goal
    # For each element `x` in the original list, format it to a 6-digit string, 
    # then convert that string back to a float.
    # pos_goal = [float(f"{x:.6f}") for x in pos_goal]

    # # 2. Update quat_goal
    # quat_goal = [float(f"{q:.6f}") for q in quat_goal]

    # rospy.loginfo(f"pos_goal: {pos_goal:.6f}")

    pose_goal = Pose()
    pose_goal.position.x = 0.374000 #pos_goal[0]
    pose_goal.position.y = 1e-6 #pos_goal[1]
    pose_goal.position.z = 0.630000 #pos_goal[2]
    pose_goal.orientation.x = 1e-6 # quat_goal[0]
    pose_goal.orientation.y = 0.7071000 # quat_goal[1]
    pose_goal.orientation.z = 1e-6 # quat_goal[2]
    pose_goal.orientation.w = 0.7071000 # quat_goal[3]

    group.set_pose_reference_frame(base_frame)
    group.set_pose_target(pose_goal)

    rospy.loginfo(f"[move_cartesian_simple] Planning to Cartesian pose in {base_frame}: "
                  f"pos={pos_goal}, quat={quat_goal}")

    # plan = group.plan()

    # if not plan or not plan.joint_trajectory.points:
    #     rospy.logwarn("[move_cartesian_simple] Planning failed or empty trajectory.")
    #     return False

    # ROS1 MoveIt (Noetic) → plan() returns (success, plan, time, error_code)
    plan_success, plan, _, _ = group.plan()

    if not plan_success or not plan or not plan.joint_trajectory.points:
        rospy.logwarn("[move_cartesian_simple] Planning failed or returned empty trajectory.")
        return False


    jt = plan.joint_trajectory
    name_to_idx = {name: i for i, name in enumerate(jt.joint_names)}
    last_pt = jt.points[-1]

    try:
        q_goal = [last_pt.positions[name_to_idx[jn]] for jn in JOINT_NAMES]
    except KeyError as e:
        rospy.logwarn(f"[move_cartesian_simple] Joint name mismatch: {e}")
        return False

    # ----------------- 2) SEND JOINT GOAL VIA EGM FOLLOWJOINTTRAJECTORY -----------
    _start_egm()
    rospy.sleep(0.2)

    pub = rospy.Publisher(JOINT_CMD_TOPIC, FollowJointTrajectoryActionGoal, queue_size=1)
    rospy.sleep(0.5)

    goal_msg = FollowJointTrajectoryActionGoal()
    traj_goal = goal_msg.goal
    traj_goal.trajectory = JointTrajectory()
    traj_goal.trajectory.joint_names = JOINT_NAMES

    pt = JointTrajectoryPoint()
    pt.positions = q_goal
    pt.time_from_start = rospy.Duration(duration)
    traj_goal.trajectory.points = [pt]
    traj_goal.goal_time_tolerance = rospy.Duration(goal_time_tolerance)

    rospy.loginfo(
        f"[move_cartesian_simple] Publishing joint goal from Cartesian plan to {JOINT_CMD_TOPIC}\n"
        f"  q_goal: {q_goal}\n"
        f"  duration: {duration}s"
    )

    pub.publish(goal_msg)

    # Rough wait for motion
    rospy.sleep(duration)

    # ----------------- 3) CHECK EE POSE WITH TF -----------------------------------
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # goal pose arrays
    p_goal = np.array(pos_goal, dtype=float)
    q_goal_arr = np.array(quat_goal, dtype=float)

    def get_ee_pose():
        try:
            tf = tf_buffer.lookup_transform(
                base_frame, ee_frame, rospy.Time(0),
                rospy.Duration(0.5)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                ROSException):
            return None, None

        p = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=float)

        q = np.array([
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ], dtype=float)

        return p, q

    def orientation_error_rad(q_curr, q_des):
        # q_err = q_des^{-1} * q_curr
        q_des_inv = tft.quaternion_inverse(q_des)
        q_err = tft.quaternion_multiply(q_des_inv, q_curr)
        w = max(min(q_err[3], 1.0), -1.0)  # clamp
        return 2.0 * math.acos(abs(w))

    success = False
    deadline = rospy.Time.now() + rospy.Duration(check_timeout)

    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        p_curr, q_curr = get_ee_pose()
        if p_curr is None:
            continue

        pos_err = np.linalg.norm(p_curr - p_goal)
        rot_err = orientation_error_rad(q_curr, q_goal_arr)

        rospy.logdebug(
            f"[move_cartesian_simple] EE check: pos_err={pos_err:.4f} m, "
            f"rot_err={rot_err:.4f} rad"
        )

        if pos_err <= pos_tolerance and rot_err <= rot_tolerance:
            rospy.loginfo(
                f"[move_cartesian_simple] Cartesian goal reached within tolerance: "
                f"pos_err={pos_err:.4f} <= {pos_tolerance}, "
                f"rot_err={rot_err:.4f} <= {rot_tolerance}"
            )
            success = True
            break

    if not success:
        rospy.logwarn(
            f"[move_cartesian_simple] EE pose NOT within tolerance after "
            f"{check_timeout:.1f}s extra. "
            f"(pos_tol={pos_tolerance}, rot_tol={rot_tolerance})"
        )

    _stop_egm()
    return success
