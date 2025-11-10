#!/usr/bin/env python3
import rospy
from com_3d.cartesian_move import execute_motion

DEFAULT_Q = [0.000, 0.720, 0.000, 0.694] # quaternion ground truth (weird that its not [0, 0, 0, 1]...)

HOME_GOAL_XYZ = [0.373, 0.000, 0.626]

# NOTE: There is a z offset due to the mounting base by 0.035 m (3.5 cm 'upwards')
PREP_GOAL_XYZ = [0.300, 0.000, 0.215] # z=0.25 in z IN WORLD
# PREP_ORIENT_Q = [0.000, 0.720, 0.000, 0.694] # Joint 5 seems to be off... This is the same orientation visually as home

PUSH_GOAL_XYZ = [0.450, 0.000, 0.215]  # move forward along x by ~15 cm


def main():
    rospy.init_node("go_forward")
    
    # push_successs = execute_motion(
    #     [PUSH_GOAL_XYZ], q_desired=DEFAULT_Q, cart_speed=0.05, eef_step=0.004, arm_logging=True)
    push_successs = execute_motion(
        [PUSH_GOAL_XYZ], cart_speed=0.05, eef_step=0.004, arm_logging=True)
    
    
    if not push_successs:
        rospy.logerr("[go_forward] Push motion failed!")
        return
    else:
        rospy.loginfo("[go_forward] succeeded.")
    
    # Now return to pre-push pose
    success_return = execute_motion(
        [PREP_GOAL_XYZ], q_desired=DEFAULT_Q, cart_speed=0.05, eef_step=0.004, arm_logging=False)
    # success_return = execute_motion(
    #     [PREP_GOAL_XYZ], cart_speed=0.05, eef_step=0.004, arm_logging=False)
    
    if not success_return:
        rospy.logerr("[go_forward] Return motion failed!")
    else:
        rospy.loginfo("[go_forward] Return to initial pose succeeded.")


if __name__ == "__main__":
    main()