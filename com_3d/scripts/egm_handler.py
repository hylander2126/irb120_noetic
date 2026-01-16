#!/usr/bin/env python3
import math
import rospy, time
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from abb_rapid_sm_addin_msgs.srv import GetEGMSettings, SetEGMSettings
from abb_robot_msgs.srv import TriggerWithResultCode


JGVC_NAME = "joint_group_velocity_controller"

SWITCH_SRV       = f"/egm/controller_manager/switch_controller"

RAPID_STOP_SRV   = f"/rws/stop_rapid"
PP_TO_MAIN_SRV       = f"/rws/pp_to_main"
RAPID_START_SRV  = f"/rws/start_rapid"

EGM_STOP_SRV     = f"/rws/sm_addin/stop_egm"
EGM_START_SRV    = f"/rws/sm_addin/start_egm_joint"
GET_SETTINGS_SRV = f"/rws/sm_addin/get_egm_settings"
SET_SETTINGS_SRV = f"/rws/sm_addin/set_egm_settings"


def shutdown_hook():
    """
    Executes automatically when Ctrl+C is pressed.
    Safely stops the ROS controller and the Robot RAPID execution.
    """
    rospy.loginfo("[EGM Manager] Shutdown signal received. Cleaning up...")

    # 1. STOP THE ROS VELOCITY CONTROLLER
    # This prevents the 'keep-alive' stream from stopping abruptly.
    try:
        # Short timeout to ensure we don't hang during shutdown
        rospy.wait_for_service(SWITCH_SRV, timeout=2.0)
        switch_ctrl = rospy.ServiceProxy(SWITCH_SRV, SwitchController)
        
        resp = switch_ctrl(
            start_controllers=[],
            stop_controllers=[JGVC_NAME],
            strictness=1, # BEST_EFFORT: won't fail if already stopped
            start_asap=False,
            timeout=1.0
        )
        if resp.ok:
            rospy.loginfo(f"[EGM Manager] {JGVC_NAME} stopped successfully.")
        else:
            rospy.logwarn(f"[EGM Manager] Failed to stop {JGVC_NAME} (might be already stopped).")
            
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"[EGM Manager] Controller Manager unavailable during shutdown: {e}")

    # 3. MOST IMPORTANT! STOP EGM
    result_code, result_message = call_trigger(EGM_STOP_SRV, timeout=1.0, SLEEP=0.5)
    rospy.loginfo(f"[EGM Manager] EGM stop result: {result_code}, message: {result_message}")

    # 2. STOP RAPID EXECUTION
    # This prevents the robot from hanging in the "EGMRunJoint" instruction.
    result_code, result_message = call_trigger(RAPID_STOP_SRV, timeout=1.0, SLEEP=0.5)
    rospy.loginfo(f"[EGM Manager] RAPID stop result: {result_code}, message: {result_message}")


def call_trigger(name, timeout=1.0, SLEEP=0.5):
    """Call a TriggerWithResultCode service and log the result."""
    try:
        rospy.wait_for_service(name, timeout=timeout)
        resp = rospy.ServiceProxy(name, TriggerWithResultCode)() # <-- Extra () to call the returned object
        
        result_code = getattr(resp, "result_code", "n/a")
        message = getattr(resp, "message", "")
        rospy.sleep(SLEEP)
        return result_code, message
    
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"[EGM_init] Service call to {name} failed: {str(e)}")
        return None, str(e)


def wait_for_egm_feedback(timeout_total=5.0):
    """Wait until we see at least one /egm/joint_states message."""
    t0 = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - t0) < timeout_total and not rospy.is_shutdown():
        try:
            rospy.wait_for_message("/egm/joint_states", JointState, timeout=0.5)
            rospy.loginfo("EGM feedback detected on /egm/joint_states.")
            return True
        except rospy.ROSException:
            pass
    rospy.logwarn("Timed out waiting for /egm/joint_states.")
    return False


def set_egm_settings():
    # Desired targets (adjust if you want):
    MAX_SPEED_DEV_RAD = 1.50 # rad/s
    COMM_TIMEOUT      = 5.00 # sec
    RAMP_IN_TIME      = 2.0 # sec # Was 0.1 then 4.0
    RAMP_OUT_TIME     = 0.25 # sec # Was 0.1

    try:
        rospy.wait_for_service(GET_SETTINGS_SRV)
        rospy.wait_for_service(SET_SETTINGS_SRV)

        get_settings = rospy.ServiceProxy(GET_SETTINGS_SRV, GetEGMSettings)
        set_settings = rospy.ServiceProxy(SET_SETTINGS_SRV, SetEGMSettings)

        s = get_settings(task='T_ROB1').settings

        s.activate.max_speed_deviation = math.degrees(MAX_SPEED_DEV_RAD) # Bump speed cap (deg/s)
        s.setup_uc.comm_timeout = COMM_TIMEOUT # Keep EGM RUNNING through brief idle
        s.run.ramp_in_time   = RAMP_IN_TIME # Snappier start/stop so abort/cancel halts immediately
        s.stop.ramp_out_time = RAMP_OUT_TIME # Snappier start/stop so abort/cancel halts immediately
        s.run.pos_corr_gain = 0.0 # Was 0.0 and worked though with traj abort errors
        # (Optional) Make RUNNING enter quickly if yours is large:
        # s.activate.cond_min_max = 0.1  # if you want a lower conditioning window

        set_resp = set_settings(task='T_ROB1', settings=s)
        result_code = getattr(set_resp, "result_code", "n/a")
        message = getattr(set_resp, "message", "")

        # if result_code == 1:
        #     # Let's show the settings we applied
        #     rospy.loginfo("EGM Settings Applied:")
            
        return result_code, message

    except rospy.ServiceException as e:
        rospy.logwarn(f"[EGM_handler] Failed to set EGM settings: {str(e)}")
        return None


def main():
    rospy.init_node("EGM_handler")

    # --- REGISTER SHUTDOWN HOOK ---
    rospy.on_shutdown(shutdown_hook)

    # 0. AS BACKUP, STOP EGM (EVEN IF EGM ISN'T RUNNING)
    # rospy.loginfo("[EGM_handler] Stopping EGM (if running)...")
    result_code, result_message = call_trigger(EGM_STOP_SRV, SLEEP=1.0)
    rospy.loginfo(f"[EGM_handler] Stopping EGM result: {result_code}, message: {result_message}")

    # 1. STOP EVERYTHING if running
    # rospy.loginfo("[EGM_handler] Stopping RAPID...")
    result_code, result_message = call_trigger(RAPID_STOP_SRV, SLEEP=1.0)
    rospy.loginfo(f"[EGM_handler] Stopping RAPID result: {result_code}, message: {result_message}")

    # # 2. SET EGM SETTINGS while RAPID is stopped
    # rospy.loginfo("[EGM_handler] Setting EGM settings...")
    # result_code, result_message = set_egm_settings()
    # rospy.loginfo(f"[EGM_handler] Set EGM settings result: {result_code}, message: {result_message}")

    # 3. RESET RAPID POINTER TO MAIN
    # rospy.loginfo("[EGM_handler] Resetting RAPID pointer to MAIN...")
    result_code, result_message = call_trigger(PP_TO_MAIN_SRV, SLEEP=1.0)
    rospy.loginfo(f"[EGM_handler] PP to Main result: {result_code}, message: {result_message}")

    # 4. START RAPID
    # rospy.loginfo("[EGM_handler] Starting RAPID...")
    result_code, result_message = call_trigger(RAPID_START_SRV, SLEEP=2.0)
    rospy.loginfo(f"[EGM_handler] Start RAPID result: {result_code}, message: {result_message}")



    # 2. SET EGM SETTINGS while RAPID is stopped
    # rospy.loginfo("[EGM_handler] Setting EGM settings...")
    result_code, result_message = set_egm_settings()
    rospy.loginfo(f"[EGM_handler] Set EGM settings result: {result_code}, message: {result_message}")
    rospy.sleep(1.0)



    # 5. SIGNAL EGM START
    # rospy.loginfo("[EGM_handler] Signaling EGM start...")
    result_code, result_message = call_trigger(EGM_START_SRV, SLEEP=1.0)
    rospy.loginfo(f"[EGM_handler] Start EGM joint result: {result_code}, message: {result_message}")
    
    # 6. SWITCH ROS CONTROLLERS
    rospy.loginfo(f"[EGM_handler] Starting {JGVC_NAME} via controller manager...")

    try:
        rospy.wait_for_service(SWITCH_SRV, timeout=5.0)
        switch = rospy.ServiceProxy(SWITCH_SRV, SwitchController)

        # Try to switch
        resp = switch(
            start_controllers=[JGVC_NAME], 
            stop_controllers=[], 
            strictness=1, # BEST_EFFORT
            start_asap=False, 
            timeout=1.0)
        
        if resp.ok:
            rospy.loginfo(f"[EGM_handler] {JGVC_NAME} successfully started")
        else:
            rospy.logerr(f"[EGM_handler] Controller Manager failed to start {JGVC_NAME}")

    except rospy.ServiceException as e:
        rospy.logerr(f"[EGM_handler] Failed to talk to Controller Manager: {e}")

    rospy.loginfo("\n[EGM_handler] COMPLETED! READY TO RECEIVE VELOCITY COMMANDS.\n")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
