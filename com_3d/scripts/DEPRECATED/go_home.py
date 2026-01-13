#!/usr/bin/env python3
import rospy
# from com_3d.cartesian_move import execute_motion
from com_3d.linear_move import execute_motion, move_joints_simple
from com_3d.cartesian_simple import move_cartesian_simple
import numpy as np


# RETRIEVED BY JOGGING ROBOT TO *EXACTLY* ALL ZERO JOINTS
HOME_XYZ = [0.374000, 0.000000, 0.630000] # TOOL FLANGE IN ROBOT FRAME (from robot, not TF) (technically 374.01, 0.01, 629.99 mm)
HOME_QUAT = [0.000000, 0.707100, 0.000000, 0.70710]

# RETRIEVED FROM ONSHAPE AND REAL MEASUREMENTS
Z_OFFSET_MOUNT = 0.035 # Z readings are 35mm higher than expected because of mounting bar
X_OFFSET_FINGER = 0.08225+ 0.11 # X readings are ~192.25 mm 'less' than expected due to FT and finger length

# ******** For DIRECT JOINT CONTROL, since sim robot is +35mm from table, joint q is already 'correct' in table frame... *********


def main():
    rospy.init_node("go_home")
    
    # motion_success = execute_motion(
    #     HOME_XYZ, q_desired=HOME_QUAT, cart_speed=0.03, eef_step=0.004, arm_logging=False)
    # motion_success = execute_motion(
    #     HOME_XYZ,
    #     cart_speed=0.03,
    #     force_stop=False,
    #     arm_logging=False
    # )

    # motion_success = move_joints_simple(
    #     [0, 0, 0, 0, 0, 0], duration=4.0, goal_time_tolerance=2.0
    # )

    motion_success = move_cartesian_simple(
        HOME_XYZ,
        HOME_QUAT,
    )
    
    if not motion_success:
        rospy.logerr("[go_home] Home motion failed!")
        return
    else:
        rospy.loginfo("[go_home] Home motion succeeded.")


if __name__ == "__main__":
    main()