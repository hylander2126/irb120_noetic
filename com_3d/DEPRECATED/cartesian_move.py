import moveit_commander
from moveit_commander import roscpp_initialize
import geometry_msgs.msg
from std_msgs.msg import Empty
from abb_robot_msgs.srv import TriggerWithResultCode
import rospy
import copy
import math
import numpy as np

MOVE_GROUP_NAME = "manipulator"
# EGM_GOAL_TOPIC = "/egm/goal"
EGM_START_SRV = "/rws/sm_addin/start_egm_joint"
EGM_STOP_SRV = "/rws/sm_addin/stop_egm"
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


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

def _arm_logs():
    rospy.Publisher('/com_3d/log_stop',  Empty, queue_size=1, latch=True).publish(Empty())
    rospy.sleep(0.05)
    rospy.Publisher('/com_3d/log_start', Empty, queue_size=1, latch=True).publish(Empty())

def _disarm_logs():
    rospy.Publisher('/com_3d/log_stop',  Empty, queue_size=1, latch=True).publish(Empty())


def _plan_cartesian_path(group: moveit_commander.MoveGroupCommander, start_pose, xyz_waypoints, q_desired, eef_step=0.004):
    
    """
    Build poses at given [[x,y,z]] waypoints while giving the option to LOCK the starting orientation.
    Returns (traj, path_len(m), fraction)
    """
    qx, qy, qz, qw = q_desired

    waypoints = []
    for x, y, z in xyz_waypoints:
        p = geometry_msgs.msg.Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.x = qx
        p.orientation.y = qy
        p.orientation.z = qz
        p.orientation.w = qw
        waypoints.append(copy.deepcopy(p))

    # Estimate geometric path length (sum of straight segments)
    path_len = 0.0
    last = (start_pose.position.x, start_pose.position.y, start_pose.position.z)
    for x, y, z in xyz_waypoints:
        path_len += math.dist(last, (x, y, z))
        last = (x, y, z)

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why eef_step = 0.01 in Cartesian translation. Disable jump 
    # threshold by setting it to 0.0 and avoid collisions = false
    (plan, fraction) = group.compute_cartesian_path(waypoints, eef_step, False) #0.0, False)
    return plan, path_len, fraction


def execute_motion(pos_desired, q_desired=np.zeros(4), cart_speed=0.03, eef_step=0.004, arm_logging=False):
    """
    Execute a Cartesian motion through given WORLD-FRAME [x,y,z] waypoint(s).

    pos_desired: list of [x,y,z] waypoints in meters
    cart_speed: desired Cartesian speed in m/s (approximate)
    eef_step: resolution of Cartesian path in meters
    """
    if not pos_desired or len(pos_desired) == 0:
        raise ValueError("No desired positions given")

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(MOVE_GROUP_NAME)

    group.set_start_state_to_current_state()
    current_pose = group.get_current_pose().pose


    # Get basic information
    print(f"========== Reference frame: {group.get_planning_frame()} ==========")

    # Print name of end-effector link
    print(f"========== End effector link: {group.get_end_effector_link()} ==========")

    # Get list of all groups in the robot
    print(f"========== Robot Groups: {robot.get_group_names()} ==========")

    print(f"\nExecuting Cartesian motion to waypoints: {pos_desired[:]}")

    ## Plan a joint goal based on the desired end x,y,z position
    if np.allclose(q_desired, np.zeros(4)):
        rospy.loginfo("Planning Cartesian path with HOLD ORIENTATION...")
        q_desired = [current_pose.orientation.x, current_pose.orientation.y,
                     current_pose.orientation.z, current_pose.orientation.w]
    else:
        rospy.loginfo(f"Planning Cartesian path with ORIENTATION: {q_desired}...")
        q_desired = q_desired

    plan, path_len, fraction = _plan_cartesian_path(group, current_pose, pos_desired, q_desired, eef_step=eef_step)
    if fraction < 0.99:
        rospy.logwarn(f"WARNING: Only {fraction*100:.1f}% of Cartesian path could be planned.")
        return False

    ## Re-time traj to be slow
    rospy.loginfo("Re-timing trajectory...")
    slow_plan = group.retime_trajectory(robot.get_current_state(), plan,
                                       velocity_scaling_factor=cart_speed,
                                       acceleration_scaling_factor=cart_speed)
    

    # ARM THE TAG, FT, and EE POSE LOGGING
    if arm_logging:
        _arm_logs()
    rospy.sleep(0.1)

    # START EGM
    _start_egm()
    rospy.sleep(0.5)
    
    # TRYING MOVEIT EXECUTE DIRECTLY (BASED ON MOVEIT.yaml)
    ok = group.execute(slow_plan, wait=True)
    rospy.loginfo("MoveIt execute returned: %s" % ok)    

    rospy.sleep(0.5)
    _stop_egm()
    if arm_logging:
        _disarm_logs()

    return ok


def print_current_pose(robot, group):
    current_pose = group.get_current_pose().pose
    rospy.loginfo(f"Current EE pose: \npos[{current_pose.position.x:.3f},{current_pose.position.y:.3f}, {current_pose.position.z:.3f}], "
                  f"\norient[{current_pose.orientation.x:.3f}, "
                  f"{current_pose.orientation.y:.3f}, "
                  f"{current_pose.orientation.z:.3f}, "
                  f"{current_pose.orientation.w:.3f}]")