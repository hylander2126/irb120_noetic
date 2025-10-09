#!/usr/bin/env python
import rospy, time, rosservice, roslib.message
from rospy.service import ServiceException

def try_switch(ns, start, stop, strictness):
    srv_name = ns + '/controller_manager/switch_controller'
    rospy.wait_for_service(srv_name, timeout=10.0)
    srv_type = rosservice.get_service_type(srv_name)
    srv_class = roslib.message.get_service_class(srv_type)
    proxy = rospy.ServiceProxy(srv_name, srv_class)

    req = srv_class._request_class()
    # Common fields in controller_manager_msgs/SwitchController
    if hasattr(req, 'start_controllers'): req.start_controllers = start
    if hasattr(req, 'stop_controllers'):  req.stop_controllers  = stop
    if hasattr(req, 'strictness'):        req.strictness        = strictness
    if hasattr(req, 'start_asap'):        req.start_asap        = False
    if hasattr(req, 'timeout'):           req.timeout           = 0.0

    resp = proxy(req)
    return getattr(resp, 'ok', True)

def main():
    rospy.init_node('switch_to_jtc')
    ns         = rospy.get_param('~ns', '/egm')
    strictness = int(rospy.get_param('~strictness', 2))
    start      = rospy.get_param('~start', ['joint_trajectory_controller'])
    stop       = rospy.get_param('~stop',  ['joint_group_velocity_controller'])
    timeout_s  = float(rospy.get_param('~timeout', 30.0))
    period_s   = float(rospy.get_param('~period', 0.5))

    t0 = time.time()
    tries = 0
    while not rospy.is_shutdown():
        tries += 1
        try:
            ok = try_switch(ns, start, stop, strictness)
            if ok:
                rospy.loginfo("Switch ok after %d tries.", tries)
                return
            else:
                rospy.logwarn("Switch returned ok=False (try %d). Retrying...", tries)
        except ServiceException as e:
            # Happens if EGM isn’t RUNNING yet or the HW refuses the claim
            rospy.logwarn("Switch failed (try %d): %s. Retrying...", tries, e)
        except Exception as e:
            rospy.logwarn("Unexpected error (try %d): %s. Retrying...", tries, e)

        if time.time() - t0 > timeout_s:
            rospy.logerr("Timed out after %.1fs trying to switch controllers.", timeout_s)
            return
        time.sleep(period_s)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
