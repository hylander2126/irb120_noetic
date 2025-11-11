#!/usr/bin/env python3
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

def _arm_logs():
    rospy.Publisher('/com_3d/log_stop',  Empty, queue_size=1, latch=True).publish(Empty())
    rospy.sleep(0.05)
    rospy.Publisher('/com_3d/log_start', Empty, queue_size=1, latch=True).publish(Empty())

def _disarm_logs():
    rospy.Publisher('/com_3d/log_stop',  Empty, queue_size=1, latch=True).publish(Empty())


# ----------------------- force monitor -----------------------
class ForceWatcher:
    """Minimal force peak tracker: detects contact → peak → triggers stop when F <= k_safe * F_peak."""
    def __init__(self, ft_topic, k_safe=0.5, contact_thresh=5.0, axis=(1,0,0)):
        self.k_safe = k_safe
        self.contact_thresh = contact_thresh # sensor noise (wavelength basically) SET THIS HIGH TO IGNORE STOP

        self.peak = 0.0
        self.in_contact = False
        self.armed = False
        self.trigger = False

        self.buf = [0.0]*5 # small rolling window
        self.below_countr = 0

        self.sub = rospy.Subscriber(ft_topic, WrenchStamped, self.cb, queue_size=50)

    def cb(self, msg):
        fx, fy, fz = msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z
        f = np.linalg.norm([fx, fy, fz])

        self.buf.pop(0)
        self.buf.append(f)
        f_med = np.median(self.buf)

        # detect contact
        if not self.in_contact:
            if f_med > self.contact_thresh:
                self.in_contact = True
            return

        # track peak once contact
        if f_med > self.peak:
            self.peak = f_med
            
        thresh = (1-self.k_safe) * self.peak

        if f_med <= thresh:
            self.below_countr += 1
            if self.below_countr >= 5:  # require 5 consecutive below-thresh readings
                self.trigger = True
                rospy.loginfo(f"\n[ForceWatcher] Triggered at force {f_med:.3f} N (peak was {self.peak:.3f} N).\n")
        else:
            self.below_countr = 0
        
        # once force is falling and we have a peak, arm
        # if self.peak > 0 and f < self.peak*0.98:  # loose drop threshold
        #     self.armed = True
        # # if armed, stop when below k_safe * peak
        # if self.armed and f <= 0.3:  # self.k_safe * self.peak:
        #     self.trigger = True
        #     rospy.loginfo(f"[ForceWatcher] Triggered at force {f:.3f} N (peak was {self.peak:.3f} N).")


# ----------------------- core API -----------------------
def execute_motion(
        xyz_list,
        force_stop=True,
        q_desired=None,
        cart_speed=0.05,
        k_safe=0.5,
        ft_topic="/netft_data_transformed",
        contact_thresh=0.3,
        arm_logging=True):

    if not xyz_list:
        rospy.logerr("[execute_motion] No XYZ passed.")
        return False

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(MOVE_GROUP_NAME)
    group.set_pose_reference_frame(POSE_REF_FRAME)

    ee = "finger_tip"
    group.set_end_effector_link(ee)
    start_pose = group.get_current_pose(ee).pose

    # IMPORTANT: set the tolerances appropriately
    group.set_goal_joint_tolerance(0.01) # rads
    group.set_goal_position_tolerance(0.005) # meters
    group.set_goal_orientation_tolerance(0.01) # rads

    # Use last waypoint only (simple, predictable)
    x,y,z = xyz_list
    goal = Pose()
    goal.position.x, goal.position.y, goal.position.z = x,y,z

    if q_desired is None:
        goal.orientation = start_pose.orientation
    else:
        goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w = q_desired

    rospy.loginfo(f"[execute_motion] Planning to: pos=({x:.3f}, {y:.3f}, {z:.3f}), \nori=({goal.orientation.x:.3f}, {goal.orientation.y:.3f}, {goal.orientation.z:.3f}, {goal.orientation.w:.3f})")

    rospy.loginfo(f"[execute_motion] Current pose: pos=({start_pose.position.x:.3f}, {start_pose.position.y:.3f}, {start_pose.position.z:.3f}), \nori=({start_pose.orientation.x:.3f}, {start_pose.orientation.y:.3f}, {start_pose.orientation.z:.3f}, {start_pose.orientation.w:.3f})   ")
    # ============================ PILZ (DOESNT WORK NOT INSTALLED)
    # if mode == "pilz":
    #     try:
    #         group.set_planning_pipeline_id("pilz_industrial_motion_planner")
    #         group.set_planner_id("LIN")
    #     except Exception:
    #         rospy.logwarn("[execute_motion] Pilz not available, falling back to OMPL.")
    #         mode = "ompl"


    # ======================== lock orientation with OMPL =======================    
    oc = OrientationConstraint()
    oc.link_name = ee
    oc.header.frame_id = POSE_REF_FRAME
    oc.orientation = goal.orientation
    oc.absolute_x_axis_tolerance = 0.02 # 0.001 radians
    oc.absolute_y_axis_tolerance = 0.02
    oc.absolute_z_axis_tolerance = 0.02
    oc.weight = 1.0

    cs = Constraints()
    cs.orientation_constraints = [oc]
    group.set_path_constraints(cs)

    group.set_pose_target(goal, ee)
    plan = group.plan()

    # Clear constraints after planning
    try:
        group.clear_path_constraints()
    except:
        pass

    # Normalize MoveIt return format
    if isinstance(plan, tuple):
        success, plan_msg, _, err = plan
        if not success:
            rospy.logerr("[execute_motion] OMPL planning failed.")
            return False
        plan = plan_msg

    if not hasattr(plan, "joint_trajectory") or not plan.joint_trajectory.points:
        rospy.logerr("[execute_motion] OMPL produced empty trajectory.")
        return False


    # Slow the speed
    slow_plan = group.retime_trajectory(
        robot.get_current_state(),
        plan,
        velocity_scaling_factor=cart_speed,
        acceleration_scaling_factor=cart_speed
    )

    plan = slow_plan

    # --------------------------------------------------------
    # Start logging & EGM
    # --------------------------------------------------------
    if arm_logging:
        _arm_logs()
    rospy.sleep(0.1)
    _start_egm()
    rospy.sleep(0.2)

    # --------------------------------------------------------
    # Force-based stop → clean cancel of MoveGroup goal
    # --------------------------------------------------------
    stop_flag = {"stop": False}
    if force_stop:
        fw = ForceWatcher(ft_topic, k_safe=k_safe, contact_thresh=contact_thresh)

        client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        client.wait_for_server()

        def watch():
            rate = rospy.Rate(300)
            while not rospy.is_shutdown():
                if fw.trigger:
                    rospy.logwarn("[execute_motion] k_safe reached → canceling MoveIt goal!")
                    stop_flag["stop"] = True
                    try:
                        client.cancel_all_goals()     # ✅ clean cancel
                        group.stop() # Ensure controller stops immediately
                        _stop_egm()
                    except Exception as e:
                        rospy.logwarn(f"[execute_motion] Exception during cancel: {e}")
                    break
                rate.sleep()


        t = threading.Thread(target=watch, daemon=True)
        t.start()
    

    # --------------------------------------------------------
    # Execute
    # --------------------------------------------------------
    ok = group.execute(plan, wait=True)
    rospy.loginfo(f"[execute_motion] MoveIt execute returned: {ok}\n")

    # --------------------------------------------------------
    # Cleanup
    # --------------------------------------------------------
    if not stop_flag["stop"] and not force_stop:
        _stop_egm()
    if force_stop:
        fw.sub.unregister()
    
    rospy.sleep(1.5)
    if arm_logging:
        _disarm_logs()

    if stop_flag["stop"]:
        rospy.loginfo("[execute_motion] Stopped early due to force fall-off.\n")
        return True

    return bool(ok)