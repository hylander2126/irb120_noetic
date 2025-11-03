#!/usr/bin/env python3
import math
import rospy, time
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from abb_rapid_sm_addin_msgs.srv import GetEGMSettings, SetEGMSettings
from abb_robot_msgs.srv import TriggerWithResultCode


def call_trigger(name, SLEEP=0.5):
    """Call a TriggerWithResultCode service and log the result."""
    rospy.wait_for_service(name)
    resp = rospy.ServiceProxy(name, TriggerWithResultCode)()
    rospy.loginfo("%s -> code=%s msg='%s'", name,
                  getattr(resp, "result_code", "n/a"),
                  getattr(resp, "message", ""))
    time.sleep(SLEEP)
    return resp


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
    MAX_SPEED_DEV_RAD = 1.5 # rad/s
    COMM_TIMEOUT      = 5.0 # sec
    RAMP_IN_TIME      = 0.1 # sec # Was 0
    RAMP_OUT_TIME     = 0.1 # sec # Was 0

    get_srv = "/rws/sm_addin/get_egm_settings"
    set_srv = "/rws/sm_addin/set_egm_settings"

    # rospy.loginfo("Setting EGM settings: Waiting for %s and %s ...", get_srv, set_srv)
    rospy.wait_for_service(get_srv)
    rospy.wait_for_service(set_srv)

    get_settings = rospy.ServiceProxy(get_srv, GetEGMSettings)
    set_settings = rospy.ServiceProxy(set_srv, SetEGMSettings)

    s = get_settings(task='T_ROB1').settings
    # s = current_settings.settings

    s.activate.max_speed_deviation = math.degrees(MAX_SPEED_DEV_RAD) # Bump speed cap (deg/s)
    s.setup_uc.comm_timeout = COMM_TIMEOUT # Keep EGM RUNNING through brief idle
    s.run.ramp_in_time   = RAMP_IN_TIME # Snappier start/stop so abort/cancel halts immediately
    s.stop.ramp_out_time = RAMP_OUT_TIME # Snappier start/stop so abort/cancel halts immediately
    s.run.pos_corr_gain = 0.0 # Was 0.0 and worked though with traj abort errors
    # (Optional) Make RUNNING enter quickly if yours is large:
    # s.activate.cond_min_max = 0.1  # if you want a lower conditioning window
    

    set_resp = set_settings(task='T_ROB1', settings=s)
    # rospy.loginfo("EGM Settings Changer: Set result: %s (%s)", set_resp.result_code, set_resp.message)
    # Read-back confirm
    # confirm = get_settings(task='T_ROB1')
    rospy.loginfo("EGM Settings Changer: Successfully updated settings!")

    time.sleep(2.0)


def main():
    # rospy.init_node("egm_settings_apply")
    rospy.init_node("egm_init")
    cm_switch_srv   = "/egm/controller_manager/switch_controller"
    jtc_name        = "joint_trajectory_controller"

    WAIT_TIME = 2.0

    rospy.loginfo("Starting EGM w/ settings and JTC init...")

    # Before doing ANYTHING, wait for EGM jtc to be available
    rospy.wait_for_service("/egm/joint_trajectory_controller/query_state")
    
    rospy.loginfo("Re-arming RAPID (stop -> pp_to_main -> start)...")
    call_trigger("/rws/stop_rapid")
    call_trigger("/rws/pp_to_main")
    call_trigger("/rws/start_rapid")

    set_egm_settings()

    rospy.loginfo("Starting EGM joint...")
    call_trigger("/rws/sm_addin/start_egm_joint")

    # Wait until EGM is actually streaming before starting JTC
    # egm_ok = wait_for_egm_feedback(timeout_total=5.0)
    # if not egm_ok:
    #     # One final nudge: try starting again once
    #     rospy.logwarn("EGM feedback not detected yet, trying to start EGM again...")
    #     call_trigger("/rws/sm_addin/start_egm_joint")
    #     egm_ok = wait_for_egm_feedback(timeout_total=5.0)

    # TODO: We should instead wait for an "OK" from the previous service call

    rospy.loginfo("Starting %s via controller manager...", jtc_name)
    rospy.wait_for_service(cm_switch_srv)
    # rospy.wait_for_service("/egm/"+jtc_name+"query_state")
    switch = rospy.ServiceProxy(cm_switch_srv, SwitchController)

    # Retry in case first attempt races EGM runtime
    for attempt in range(1,6):
        try:
            switch(start_controllers=[jtc_name], 
                stop_controllers=[], 
                strictness=2,
                start_asap=False, 
                timeout=0.0)
            rospy.loginfo("JTC started")
            break
        except rospy.ServiceException as e:
            rospy.logwarn("SwitchController attempt %d failed: %s", attempt, str(e))
            time.sleep(0.4)
    else: # only executes if no "break" from loop encountered
        rospy.logerr("Failed to start JTC after several attempts, aborting.")
        return


    rospy.loginfo("Init complete: EGM running, JTC started. Ready to receive trajectories.")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
