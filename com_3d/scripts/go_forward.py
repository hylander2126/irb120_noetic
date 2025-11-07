#!/usr/bin/env python3
import rospy
from com_3d.cartesian_move import execute_motion

# NOTE:
#--- Pre-Push Pose ---
# position: [0.280734, -0.000912, 0.237589]
# orientation: [0.001030, 0.712448, -0.001284, 0.701723]

# R = [ -0.0151676,  0.0032697,  0.9998796;
#       -0.0003344,  0.9999946, -0.0032751;
#       -0.9998849, -0.0003840, -0.0151664 ]
# ------------------------------
# From TF:
# trans: 0.290 0, 0.244
# rot rpy rad: 0, 0.015, 0.003
INIT_POSE_XYZ = [0.280734, -0.000912, 0.237589]
PUSH_GOAL_XYZ = [0.430734, -0.000912, 0.237589]  # move forward along x by ~15 cm


def main():
    rospy.init_node("go_forward")
    
    push_successs = execute_motion(
        [PUSH_GOAL_XYZ], cart_speed=0.05, eef_step=0.004, arm_logging=True)
    
    if not push_successs:
        rospy.logerr("[go_forward] Push motion failed!")
        return
    else:
        rospy.loginfo("[go_forward] succeeded.")
    
    # Now return to pre-push pose
    success_return = execute_motion(
        [INIT_POSE_XYZ], cart_speed=0.05, eef_step=0.004, arm_logging=False)
    
    if not success_return:
        rospy.logerr("[go_forward] Return motion failed!")
    else:
        rospy.loginfo("[go_forward] Return to initial pose succeeded.")


if __name__ == "__main__":
    main()