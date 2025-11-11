#!/usr/bin/env python3
import rospy
from com_3d.linear_move import execute_motion

DEFAULT_Q = [0.000, 0.7313537, 0.000, 0.6819984] # manually determined for better alignment for now

HOME_GOAL_XYZ = [0.373, 0.000, 0.626]

# NOTE: There is a z offset due to the mounting base by 0.035 m (3.5 cm 'upwards')
PREP_GOAL_XYZ = [0.440, 0.000, 0.215] # z=0.25 in z IN WORLD

PUSH_GOAL_XYZ = [0.600, 0.000, 0.215]  # move forward along x by ~15 cm

# TF ORI CORRECT = [0.003, 0.717, 0.001, 0.697]
# MUST COMMAND   = [0, 0.7253744, 0, 0.6883546]




def main():
    rospy.init_node("go_forward")

    push_success = execute_motion(
        PUSH_GOAL_XYZ,
        cart_speed=0.03,
        k_safe=0.90,                # stop when force <= 60% of peak
        ft_topic="netft_data_transformed",    # change to your topic
        contact_thresh=0.4, # Should be higher than sensor noise (when not in contact)
        arm_logging=True
    )

    if not push_success:
        rospy.logerr("[go_forward] Push motion failed!")
        return
    else:
        rospy.loginfo("[go_forward] succeeded.")

    # Now return to pre-push pose
    success_return = execute_motion(
        PREP_GOAL_XYZ,
        cart_speed=0.03,
        force_stop=False,
        ft_topic="netft_data_transformed",
        arm_logging=False
    )

    if not success_return:
        rospy.logerr("[go_forward] Return motion failed!")
    else:
        rospy.loginfo("[go_forward] Return to initial pose succeeded.")


if __name__ == "__main__":
    main()