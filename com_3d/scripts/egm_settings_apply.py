#!/usr/bin/env python3
import math
import rospy
from abb_rapid_sm_addin_msgs.srv import GetEGMSettings, SetEGMSettings

def main():
    rospy.init_node("egm_settings_apply")

    # Desired targets (adjust if you want):
    max_speed_dev_rad = 1.5 # rad/s
    comm_timeout      = 5.0 # sec
    ramp_in_time      = 0.1 # sec # Was 0
    ramp_out_time     = 0.1 # sec # Was 0

    get_srv = "/rws/sm_addin/get_egm_settings"
    set_srv = "/rws/sm_addin/set_egm_settings"

    rospy.loginfo("EGM Settings Changer: Waiting for %s and %s ...", get_srv, set_srv)
    rospy.wait_for_service(get_srv)
    rospy.wait_for_service(set_srv)

    # get_settings = rospy.ServiceProxy(get_srv, SetEGMSettings._request_class._type_support.get_service_class('abb_rapid_sm_addin_msgs/GetEGMSettings'))
    get_settings = rospy.ServiceProxy(get_srv, GetEGMSettings)
    set_settings = rospy.ServiceProxy(set_srv, SetEGMSettings)

    # Use the proper service classes explicitly (works across msg gens)
    # get_settings = rospy.ServiceProxy(get_srv, GetEGMSettings)

    current_settings = get_settings(task='T_ROB1')
    s = current_settings.settings
    # rospy.loginfo("EGM Settings Changer: Current settings:\n%s", s)

    # Bump speed cap (deg/s)
    s.activate.max_speed_deviation = math.degrees(max_speed_dev_rad)
    # Keep EGM RUNNING through brief idle
    s.setup_uc.comm_timeout = comm_timeout
    # Snappier start/stop so abort/cancel halts immediately
    s.run.ramp_in_time   = ramp_in_time
    s.stop.ramp_out_time = ramp_out_time

    s.run.pos_corr_gain = 0.0 # Was 0.0 and worked though with traj abort errors

    # (Optional) Make RUNNING enter quickly if yours is large:
    # s.activate.cond_min_max = 0.1  # if you want a lower conditioning window
    
    set_resp = set_settings(task='T_ROB1', settings=s)
    rospy.loginfo("EGM Settings Changer: Set result: %s (%s)", set_resp.result_code, set_resp.message)

    # Read-back confirm
    confirm = get_settings(task='T_ROB1')
    # rospy.loginfo("EGM Settings Changer: Confirmed settings:\n%s", confirm.settings)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
