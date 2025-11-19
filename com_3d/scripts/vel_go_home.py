#!/usr/bin/env python3
import rospy
import numpy as np
from com_3d.velocity_controller import VelocityCommander

# RETRIEVED BY JOGGING ROBOT TO *EXACTLY* ALL ZERO JOINTS
HOME_XYZ = [0.374000, 0.000000, 0.630000] # TOOL FLANGE IN ROBOT FRAME (from robot, not TF) (technically 374.01, 0.01, 629.99 mm)
HOME_QUAT = [0.000000, 0.707100, 0.000000, 0.70710]

# RETRIEVED FROM ONSHAPE AND REAL MEASUREMENTS
Z_OFFSET_MOUNT = 0.035 # Z readings are 35mm higher than expected because of mounting bar
X_OFFSET_FINGER = 0.08225+ 0.11 # X readings are ~192.25 mm 'less' than expected due to FT and finger length

# ******** For DIRECT JOINT CONTROL, since sim robot is +35mm from table, joint q is already 'correct' in table frame... *********


def main():
    rospy.init_node("go_home")

    commander = VelocityCommander()

    rospy.loginfo("[go_home] Moving robot to home position using velocity control...")
    # motion_success = commander.go_home_velocity(timeout=20.0)
    # motion_success = commander.go_to_joint_target([0, 0.615, 0.867, 0, -1.482, 0], timeout=20.0) # we know this is prepush pose for "box"

    motion_success = commander.move_cartesian_velocity(
        [0, 0.1, 0],
        [0, 0, 0],
        duration=1,
    )
    
    if not motion_success:
        rospy.logerr("[go_home] Home motion failed!")
        return
    else:
        rospy.loginfo("[go_home] Home motion succeeded.")


if __name__ == "__main__":
    main()