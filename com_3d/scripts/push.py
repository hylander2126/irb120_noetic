#!/usr/bin/env python3
import rospy
from com_3d.vel_controller import VelocityController
from com_3d.force_watcher import ForceWatcher
from std_msgs.msg import Float64MultiArray, Empty, Bool
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


class EstimateListener:
    """
    Helper to listen for online estimates reliably.
    Prevents race conditions by maintaining a persistent subscriber.
    """
    def __init__(self):
        self._data = None
        self._received = False
        # Persistent subscriber ensures we don't miss unlatched messages
        self._sub = rospy.Subscriber("/com_3d/online_estimate", Float64MultiArray, self._cb)

        self._arm_log_pub = rospy.Publisher('/com_3d/log_start', Empty, queue_size=1, latch=True)
        self._disarm_log_pub = rospy.Publisher('/com_3d/log_stop',  Empty, queue_size=1, latch=True)
        self._arm_est_pub = rospy.Publisher('/com_3d/est_start', Empty, queue_size=1, latch=True)
        self._disarm_est_pub = rospy.Publisher('/com_3d/est_stop',  Empty, queue_size=1, latch=True)

        self._retract_pub = rospy.Publisher('/com_3d/retract_phase', Bool, queue_size=1, latch=False)

    def _cb(self, msg):
        self._data = msg.data
        self._received = True

    def clear(self):
        """Call this BEFORE triggering the estimator."""
        self._received = False
        self._data = None

    def wait_for_new_estimate(self, timeout=3.0):
        """Call this AFTER triggering the estimator."""
        start = rospy.Time.now()
        rate = rospy.Rate(20) # Check 20 times a second
        
        out = None

        while not rospy.is_shutdown():
            if self._received:
                out = self._data
                m_est, zc_est, theta_star = out
                rospy.loginfo(f"[push] Received online estimate data: m={m_est:.3f} kg, zc={zc_est:.3f} m, theta*={np.degrees(theta_star):.2f} deg")
                return out
            
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("[push] No online estimate available / received.")
                return [None, None, None]
            
            rate.sleep()

    def arm_logs(self):
        self._arm_log_pub.publish(Empty())

    def arm_estimate(self):
        self._arm_est_pub.publish(Empty())

    def disarm_logs(self):
        self._disarm_log_pub.publish(Empty())

    def disarm_estimate(self):
        self._disarm_est_pub.publish(Empty())
    

def check_manip_success(success: bool, action_desc: str) -> bool:
    """Check if a manipulation action succeeded, log and return."""
    if not success:
        rospy.logerr(f"[push] {action_desc} motion failed!")
        return False
    rospy.loginfo(f"[push] {action_desc} succeeded.\n")
    return True


def choose_second_push(zc_est, margin, obj_ht):
    """
    Decide whether to do a second push based on current estimate and safety factor.
    """
    assert 0.0 <= margin <= 1.0, "Margin must be between 0 and 1."
    next_push_ht = zc_est + (zc_est * margin)

    next_push_ht = np.clip(next_push_ht, 0.065, obj_ht) # min just above floor z safety (0.06) constraint, max object height

    return next_push_ht
    

def prepush_procedure(controller, position, quaternion, joint_tol, retries=0):
    """
    Procedure to prepare the robot for pushing.
    """
    for attempt in range(retries + 1):
        rospy.loginfo(f"[push] Moving to pre-push pose at {position} (Attempt {attempt + 1}/{retries + 1})...")
        
        success_prep = controller.move_to_pose(position, quaternion, timeout=8.0, tol=joint_tol)
        if not check_manip_success(success_prep, f"Pre-push Attempt {attempt + 1}"):
            # If last attempt, raise error and shutdown this node
            if attempt == retries:
                rospy.logerr(f"[push] Failed to reach pre-push pose after {attempt+1}/{retries + 1} attempts. Aborting.")
                rospy.signal_shutdown("Pre-push pose unreachable.")
                return False
        else:
            break
    return True


def execute_push_sequence(ctrl, pos, quat, fw, est, push_speed, duration, joint_tol, enable_logging, retries):
    """
    Execute complete push sequence: prep -> push -> retract.
    
    Args:
        enable_logging: If True, start/stop logging around this push sequence
    """
    success_prepush = prepush_procedure(ctrl, pos, quat, joint_tol, retries)
    if not success_prepush:
        est.disarm_logs()  # Ensure logging is stopped if max retries exceeded
        return None
    
    fw.reset()  # Reset static variables in ForceWatcher (Backstop to prevent booleans sticking)
    fw.is_active = True
    est.clear()  # Clear any old estimates
    rospy.sleep(1.0) # NICE LONG SLEEP TO LET MOTIONS FINISH AND STABILIZE
    est.arm_estimate()  # Start estimator

    if enable_logging:
        est.arm_logs()  # Start logging for push motion

    success_push = ctrl.cartesian_velocity(
        v=[push_speed, 0, 0], # XYZ
        w=[0, 0, 0],   # RPY
        duration=duration,
        force_watcher=fw,
        lock_orient=True,
    )
    # disarm_estimate()  # Stop estimator
    # fw.is_active = False
    if not check_manip_success(success_push, "Push"):
        est.disarm_logs()  # Ensure logging is stopped
        return None
    
    # Pull the most recent streaming estimate (if available)
    # data = est.wait_for_new_estimate(timeout=3.0) # Returns None, None, None if no estimate available
    
    # est.publish_retract(True)  # Notify estimator of retract phase
    # rospy.sleep(0.1)  # Small delay to ensure retract phase is registered

    success_retract = ctrl.cartesian_velocity(
        v=[-push_speed, 0, 0], # XYZ
        w=[0, 0, 0],   # RPY
        duration=ctrl.last_cartesian_duration + 0.2, # returns ~same distance + 0.2 buffer to be safe
        force_watcher=None, # No force watcher on retract
        lock_orient=True,
        pub_retract=True,
    )
    # est.publish_retract(False)  # Notify estimator retract phase is over
    fw.is_active = False
    est.disarm_estimate()  # Stop estimator
    data = est.wait_for_new_estimate(timeout=3.0) # Returns None, None, None if no estimate available
    if not check_manip_success(success_retract, "Retraction"):
        est.disarm_logs()  # Ensure logging is stopped
        return None

    return data  # m_est, zc_est, theta_star


def main():
    rospy.init_node("push")

    est = EstimateListener()

    DURATION    = 12.0#22.0 # 12.0 secs # I halved the push speed, so have to double duration
    PUSH_SPEED  = 0.01#0.005 # 0.01 m/s
    JOINT_TOL   = 2e-3 # rad

    n_SAFETY = rospy.get_param("~n_safety", -1) # (1= full safety (stop upon contact), 0= full topple)
    object_name = rospy.get_param("~object", None)
    if object_name is None or object_name not in OBJECT_MOTIONS:
        rospy.signal_shutdown(f"Invalid object name. Set _object:= to one of {list(OBJECT_MOTIONS.keys())}")
    if not (0.0 <= n_SAFETY <= 1.0):
        rospy.signal_shutdown(f"[push] n_safety must be in [0,1], got n_safety={n_SAFETY}")
    
    rospy.loginfo(f"[push] Using n_safety={n_SAFETY:.2f}")
    

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
    rospy.sleep(0.5) # Let things settle for baseline collection


    try:
        ## =================== BEGIN MOTION SEQUENCE ===================== 
        data = execute_push_sequence(
            ctrl=ctrl, 
            pos=pos, 
            quat=quat, 
            fw=fw, 
            est=est, 
            push_speed=PUSH_SPEED, 
            duration=DURATION, 
            joint_tol=JOINT_TOL, 
            enable_logging=True, # START logging
            retries=0 # No retries for first push (fail fast if pose unreachable)
        )
        if data is None:
            rospy.signal_shutdown("Push sequence failed.")
            return
        else:
            m_est, zc_est, theta_star = data


        return

        # *********************************************
        # ADAPTIVE SECOND PUSH DECISION BASED ON ESTIMATE
        if m_est is None or zc_est is None:
            rospy.logwarn("[push] No online estimate available, returning to initial pose.")
            prepush_procedure(ctrl, pos, quat, JOINT_TOL, retries=0)
            est.disarm_logs()  # Ensure logging is stopped
            rospy.signal_shutdown("No estimate available after first push.")
        else:
            margin = 0.1 # 10% margin
            next_push_ht = choose_second_push(zc_est, margin=margin, obj_ht=height)
            rospy.loginfo(f"[push] Next push height selected at zc + {100*margin}% margin: {next_push_ht:.4f} m")

        # 5) Move to next pre-push height (with one retry)
        data = execute_push_sequence(
            ctrl=ctrl, 
            pos=[pos[0], pos[1], next_push_ht], 
            quat=quat, 
            fw=fw, 
            est=est, 
            push_speed=PUSH_SPEED, 
            duration=DURATION, 
            joint_tol=JOINT_TOL, 
            enable_logging=False, # STOP logging
            retries=1 # One retry for second push to allow for minor corrections
        )

        # 7) Return to original pre-push pose
        prepush_procedure(ctrl, pos, quat, JOINT_TOL, retries=1)
    finally:
        rospy.sleep(0.5)
        est.disarm_logs()  # Ensure logging is stopped AFTER ALL MOTIONS
        rospy.sleep(0.5)
        return



if __name__ == "__main__":
    main()