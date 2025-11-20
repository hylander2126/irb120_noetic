#!/usr/bin/env python3
import rospy
from com_3d.vel_controller import VelocityController

# RETRIEVED BY JOGGING ROBOT TO *EXACTLY* ALL ZERO JOINTS
HOME_XYZ = [0.374000, 0.000000, 0.630000] # TOOL FLANGE IN ROBOT FRAME (from robot, not TF) (technically 374.01, 0.01, 629.99 mm)
HOME_QUAT = [0.000000, 0.707100, 0.000000, 0.70710]

# RETRIEVED FROM ONSHAPE AND REAL MEASUREMENTS
Z_OFFSET_MOUNT = 0.035 # Z readings are 35mm higher than expected because of mounting bar
X_OFFSET_FINGER = 0.08225+ 0.11 # X readings are ~192.25 mm 'less' than expected due to FT and finger length

# ******** For DIRECT JOINT CONTROL, since sim robot is +35mm from table, joint q is already 'correct' in table frame... *********


def main():
    rospy.init_node("go_home")

    ctrl = VelocityController(max_joint_vel=1.0)

    rospy.loginfo("[go_home] Moving robot to home position using velocity control...")
    # motion_success = ctrl.cartesian_velocity(v=[0, 0, -0.01], w=[0, 0, 0], duration=2.0)
    motion_success = ctrl.move_to_joint_positions(
        [0, 0, 0, 0, 0, 0], 
        timeout=15.0,
        bypass_floor=True
        )
    
    if not motion_success:
        rospy.logerr("[go_home] Home motion failed!")
        return
    else:
        rospy.loginfo("[go_home] Home motion succeeded.")


if __name__ == "__main__":
    main()