#!/usr/bin/env python3
import rospy
from com_3d.vel_controller import VelocityController
from com_3d.force_watcher import ForceWatcher
from std_msgs.msg import Float64MultiArray, Empty
import numpy as np

# RETRIEVED BY JOGGING ROBOT TO *EXACTLY* ALL ZERO JOINTS ( THESE ARE ALL FROM ROBOT TRANSFORM ON TABLET)
# HOME_XYZ = [0.374000, 0.000000, 0.630000] # TOOL FLANGE
# HOME_QUAT = [0.000000, 0.7071000, 0.000000, 0.7071000]

# RETRIEVED FROM ONSHAPE AND REAL MEASUREMENTS
# Z_OFFSET_MOUNT = 0.035 # Z readings are 35mm higher than expected because of mounting bar
# X_OFFSET_FINGER = 0.081+ 0.114 # X readings are ~192.25 mm 'less' than expected due to FT and finger length

HOME_XYZ = [0.569, 0.0, 0.640] # The hardcorded numbers are the TRUE home pos of FLANGE in TABLE FRAME
HOME_QUAT = [0.0, 0.70817, 0.0, 0.70604]

O_OBJ = [0.47065 + 0.081 + 0.114, 0, 0] # Location of object frame in robot (table) frame

# ******** For DIRECT JOINT CONTROL, since sim robot is +35mm from table, joint q is already 'correct' in table frame... *********
### AKA NO NEED TO CORRECT THE Z HEIGHT AT ALL... JUST X

# -----------------------------------------------------------------------------
# PER-OBJECT CARTESIAN GOALS (FINGER TIP COORDINATES wrt ROBOT/TABLE FRAME)
# -----------------------------------------------------------------------------

OBJECT_MOTIONS = {
    "box": {
        "prep_xyz": [0.5173, 0.0, 0.28], # = 0.3 (box ht but minus the bracket z offset)], # Corresponds to prep_q below
        "prep_quat": HOME_QUAT,
        # "prep_q": [0, 0.4715, 0.8937, 0, -1.3647, 0],
        # [0, 0.615, 0.867, 0, -1.482, 0], # manual prep joint config ( Yields Z=225mmm..?)

        "com": [-0.04515, 0.0, 0.14624], # -0.0500, 0, 0.1500]
        "mass": 0.664, # 635
        "theta_star": 0.0, # placeholder
        "height": 0.3,
        "est": [0,0,0],
    },
    
    "heart":{
        "prep_xyz": [0.5164, 0.0, 0.18], #0.16], #0.152], # Corresponds to prep_q below
        "prep_quat": HOME_QUAT,
        # "prep_q": [0, 0.8664, 0.7819, 0, -1.6441, 0],
        # [0, 0.941, 0.748, 0, -1.689, 0],

        "com": [-0.04354, 0, 0.098],
        "mass": 0.236, # 0.269
        "theta_star": 0.0, # placeholder
        "height": 0.2,
        "est": [0,0,0],
    },

    # "lshape": {
    #     "prep_q": [0, 1.091, 0.506, 0, -1.597, 0] # manual prep joint config
    # },

    "monitor": {
        "prep_xyz": [0.5667, 0.0, 0.475], #0.4595], # Corresponds to prep_q below
        "prep_quat": HOME_QUAT,
        # "prep_q": [0, 0.054, 0.552, 0.006, -0.608, 0],

        "com": [-0.06207, 0.0, 0.2516], # [-0.0781, 0, 0.2362]
        "mass": 5.008,
        "theta_star": 0.0, # placeholder
        "height": 0.49828,
        "est": [0,0,0],
    },

    "flashlight": {
        "prep_xyz": [0.5667, 0.0, 0.1734], #0.1534], # Corresponds to prep_q below
        "prep_quat": HOME_QUAT,
        # "prep_q": [0, 0.978, 0.566, 0, -1.544, 0], # manual prep joint config

        "com": [-0.0230, 0.0, 0.09656], # [-0.0250, 0, 0.0950]
        "mass": 0.387,
        "theta_star": 0.0, # placeholder
        "height": 0.2,
        "est": [0,0,0],
    },

    # "soda": {
    #     "prep_q": [0, 0.803, 0.640, 0, -1.445, 0] # manual prep joint config
    # }
}


def _wait_and_report_online_estimate(timeout_s: float = 3.0):
    """Wait for the latest online estimate and print it nicely."""
    try:
        msg = rospy.wait_for_message("/com_3d/online_estimate", Float64MultiArray, timeout=timeout_s)
    except rospy.ROSException:
        rospy.logwarn("[push] No /com_3d/online_estimate received (is streaming_estimator running?).")
        return

    m_est, zc_est, theta_star = msg.data[:3]
    rospy.loginfo(
        "[push] FINAL ONLINE ESTIMATE: m=%.4f kg, zc=%.4f m, theta*=%.2f deg",
        m_est, zc_est, np.rad2deg(theta_star)
    )
    # also stash for other nodes / debugging
    rospy.set_param("/com_3d/last_online_estimate", [float(m_est), float(zc_est), float(theta_star)])

    return [m_est, zc_est, theta_star]


def arm_logs():
    rospy.Publisher('/com_3d/log_start', Empty, queue_size=1, latch=True).publish(Empty())

def disarm_logs():
    rospy.Publisher('/com_3d/log_stop',  Empty, queue_size=1, latch=True).publish(Empty())

def check_manip_success(success: bool, action_desc: str) -> bool:
    """Check if a manipulation action succeeded, log and return."""
    if not success:
        rospy.logerr(f"[push] {action_desc} motion failed!")
        return False
    rospy.loginfo(f"[push] {action_desc} succeeded.\n")
    return True

def choose_second_push(zc_est, margin, obj_ht, n_safety):
    """
    Decide whether to do a second push based on current estimate and safety factor.
    """
    assert 0.0 <= margin <= 1.0, "Margin must be between 0 and 1."
    next_push_ht = zc_est + (zc_est * margin)

    next_push_ht = np.clip(next_push_ht, 0.2, obj_ht) # min 20cm, max object height

    return next_push_ht
    


def main():
    rospy.init_node("push")

    DURATION    = 12.0 # secs 8.0 # seconds
    PUSH_SPEED  = 0.01 # m/s
    JOINT_TOL   = 2e-3 # rad

    n_SAFETY = rospy.get_param("~n_safety", 0.9) # (1= full safety (stop upon contact), 0= full topple)
    rospy.loginfo(f"[push] Using n_safety={n_SAFETY:.2f}")

    object_name = rospy.get_param("~object", None)
    if object_name is None or object_name not in OBJECT_MOTIONS:
        rospy.logerr(f"[push] Object '{object_name}' not recognized. Set _object:= to one of: {list(OBJECT_MOTIONS.keys())}")
        return
    

    # Publish some params globally for estimation and logging
    pos         = OBJECT_MOTIONS[object_name]["prep_xyz"]
    quat        = OBJECT_MOTIONS[object_name]["prep_quat"]

    com         = OBJECT_MOTIONS[object_name]["com"]
    height      = OBJECT_MOTIONS[object_name]["height"]
    theta_star  = np.arctan2(np.linalg.norm(com[:2]), com[2])

    rospy.set_param("/com_3d/object_name", object_name)
    rospy.set_param("/com_3d/o_obj", list(O_OBJ))
    rospy.set_param("/com_3d/rc0_known", list([com[0], com[1], 0.0]))
    rospy.set_param("/com_3d/theta_star", float(theta_star))

    rospy.loginfo(f"[push] Preparing to push object '{object_name}'.")

    # Velocity controller
    ctrl = VelocityController(max_joint_vel=1.0)
    # Force watcher
    fw = ForceWatcher(
        n_safety=n_SAFETY,
        debug=True,               # ENABLE DEBUGGING
        initial_state="BASELINE", # THIS WAS NONE BEFORE, and we set Baseline INSIDE the motion loop
    )
    rospy.sleep(0.25) # Let things settle for baseline collection


    ## =================== BEGIN MOTION SEQUENCE ===================== 
    # 1) Move to pre-push pose
    success_prep = ctrl.move_to_pose(pos, quat, timeout=8.0, tol=JOINT_TOL)
    if not check_manip_success(success_prep, "Pre-push"):
        return
    

    # 2) ********** Execute push motion ***********
    rospy.sleep(1.5) # NICE LONG SLEEP TO LET MOTIONS FINISH AND STABILIZE
    fw.reset()  # Reset static variables in ForceWatcher (Backstop to prevent booleans sticking)
    arm_logs()  # Start logging for push motion

    success_push = ctrl.cartesian_velocity(
        v=[PUSH_SPEED, 0, 0], # XYZ
        w=[0, 0, 0],   # RPY
        duration=DURATION, # 8
        force_watcher=fw,
        lock_orient=True,
    )
    disarm_logs()  # Stop logging for push motion
    if not check_manip_success(success_push, "Push"):
        return
    
    # Pull the most recent streaming estimate (if available)
    try:
        m_est, zc_est, theta_star = _wait_and_report_online_estimate(timeout_s=3.0)
    except:
        m_est, zc_est, theta_star = None, None, None

    # *********************************************


    # 3) Retract along same linear path
    success_retract = ctrl.cartesian_velocity(
        v=[-PUSH_SPEED, 0, 0], # XYZ
        w=[0, 0, 0],   # RPY
        duration=ctrl.last_cartesian_duration, # returns ~same distance
        lock_orient=True,
    )
    if not check_manip_success(success_retract, "Retraction"):
        return


    # ================================================
    # ADAPTIVE SECOND PUSH DECISION BASED ON ESTIMATE
    # ================================================

    # 4) If no estimate, skip second push, return to original pose
    if m_est is None or zc_est is None:
        rospy.logwarn("[push] No online estimate available, skipping second push decision.")

        success_return = ctrl.move_to_pose(pos, quat, timeout=8.0, tol=JOINT_TOL)
        check_manip_success(success_return, "Return")
        return
    

    # 5) If estimate available, decide on next push height, and execute
    arm_logs()  # Start logging for second push motion
    margin = 0.1 # 10% margin
    JOINT_TOL  = 4e-3 # rad
    next_push_ht = choose_second_push(zc_est, margin=margin, obj_ht=height, n_safety=n_SAFETY)

    rospy.loginfo(f"[push] Next push height selected at zc + {100*margin}% margin: {next_push_ht:.4f} m")

    success_return = ctrl.move_to_pose(
        xyz=[pos[0], pos[1], next_push_ht],
        quat=quat,
        timeout=8.0,
        tol=JOINT_TOL
    )
    if not check_manip_success(success_return, "Active Push Return"):
        return
    
    success_acctive = ctrl.cartesian_velocity(
        v=[PUSH_SPEED, 0, 0], # XYZ
        w=[0, 0, 0],   # RPY
        duration=DURATION, # 8
        force_watcher=fw,
        lock_orient=True,
    )
    disarm_logs()  # Stop logging for second push motion
    if not check_manip_success(success_acctive, "Active Second Push"):
        return

     # Pull the most recent streaming estimate (if available)
    try:
        m_est, zc_est, theta_star = _wait_and_report_online_estimate(timeout_s=3.0)
    except:
        m_est, zc_est, theta_star = None, None, None



if __name__ == "__main__":
    main()