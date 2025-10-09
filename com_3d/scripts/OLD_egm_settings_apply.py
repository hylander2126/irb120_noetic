
#!/usr/bin/env python3
import math
import rospy
from abb_rapid_sm_addin_msgs.srv import *

def set_if_exists(obj, dotted_name, value):
    parts = dotted_name.split(".")
    cur = obj
    for p in parts[:-1]:
        if not hasattr(cur, p):
            return False
        cur = getattr(cur, p)
    leaf = parts[-1]
    if hasattr(cur, leaf):
        setattr(cur, leaf, value)
        return True
    return False

def main():
    rospy.init_node("egm_settings_apply")

    # Allow remapping/params
    ns = rospy.get_param("~ns", "/rws/sm_addin")
    task = rospy.get_param("~task", "T_ROB1")
    max_speed_dev_rad = rospy.get_param("~max_speed_deviation_rad_s", 3.0)  # rad/s -> will convert to deg/s

    get_srv_name = ns + "/get_egm_settings"
    set_srv_name = ns + "/set_egm_settings"

    rospy.loginfo("EGM Settings Changer: -------- INITIALIZING --------")
    rospy.loginfo("EGM Settings Changer: Waiting for services: %s, %s", get_srv_name, set_srv_name)
    rospy.wait_for_service(get_srv_name)
    rospy.wait_for_service(set_srv_name)

    get_settings = rospy.ServiceProxy(get_srv_name, GetEGMSettings)
    set_settings = rospy.ServiceProxy(set_srv_name, SetEGMSettings)

    # Read current
    resp = get_settings(task=task)
    settings = resp.settings
    rospy.loginfo("EGM Settings Changer: Current EGM settings:\n%s", settings)

    # Modify settings (only if fields exist in your controller’s schema)
    changed = False

    # 1) Allow higher tracking speeds (deg/s)
    if set_if_exists(settings, "activate.max_speed_deviation", math.degrees(max_speed_dev_rad)):
        changed = True
        rospy.loginfo("EGM Settings Changer: Set activate.max_speed_deviation = %.1f deg/s", math.degrees(max_speed_dev_rad))

    # 2) Make RUNNING more robust: shorter conditioning, longer comm timeout (if your schema exposes them)
    # Common field names vary between SM Add-In versions. We try both typical spots.
    if set_if_exists(settings, "activate.cond_time", 0.05):
        changed = True
        rospy.loginfo("EGM Settings Changer: Set activate.cond_time = 0.05 s")
    if set_if_exists(settings, "run.comm_timeout", 5.0):
        changed = True
        rospy.loginfo("EGM Settings Changer: Set run.comm_timeout = 5.0 s")

    # (Optional) Soften ramp-out to avoid long coasts on abort; or set 0 for immediate stop if available
    if set_if_exists(settings, "run.ramp_out_time", 0.0):
        changed = True
        rospy.loginfo("EGM Settings Changer: Set run.ramp_out_time = 0.0 s")

    if not changed:
        rospy.logwarn("EGM Settings Changer: No known fields were updated (schema didn’t contain expected members).")
        return

    # Write back
    rospy.loginfo("EGM Settings Changer: Writing EGM settings back to task %s ..", task)
    set_settings(task=task, settings=settings)
    rospy.loginfo("EGM Settings Changer: EGM settings applied.")

    # Read once more to confirm
    confirm = get_settings(task=task)
    rospy.loginfo("EGM Settings Changer: Confirmed EGM settings:\n%s", confirm.settings)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
