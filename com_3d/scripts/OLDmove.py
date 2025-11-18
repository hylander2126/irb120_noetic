#!/usr/bin/env python3
import rospy
from com_3d.linear_move import execute_motion, go_to_pose_simple

DEFAULT_Q = [0.000, 0.7313537, 0.000, 0.6819984] # manually determined for better alignment for now
HOME_GOAL_XYZ = [0.373, 0.000, 0.626]

DEFAULT_OBJECT = "box"

# -----------------------------------------------------------------------------
# PER-OBJECT CARTESIAN GOALS (note that z=0 is actually z=0.035 due to the mounting base)
# -----------------------------------------------------------------------------
Z_OFFSET = 0.035  # mounting base height

OBJECT_MOTIONS = {
    "box": {
        "prep": [0.440, 0.000, 0.25-Z_OFFSET],  # z=0.25 in z IN WORLD
        "push": [0.600, 0.000, 0.25-Z_OFFSET],  # move forward along x by ~15 cm
        "quat": [0.003, 0.717, 0.001, 0.697]
    },
    
    "heart":{
        "prep": [0.440, 0.000, 0.25-Z_OFFSET],  # z=0.25 in z IN WORLD
        "push": [0.600, 0.000, 0.25-Z_OFFSET],  # move forward along x by ~15 cm
        "quat": [0.003, 0.717, 0.001, 0.697] # TODO: update this one
    },

    "lshape": {
        "prep": [0.440, 0.000, 0.13-Z_OFFSET],  # z=0.13 in z IN WORLD
        "push": [0.600, 0.000, 0.13-Z_OFFSET],  # move forward along x by ~15 cm
        "quat": [-0.001, 0.703, 0.005, 0.711] # TODO : update this one
    },

    "monitor": {
        "prep": [0.440, 0.000, 0.50-Z_OFFSET],  # z=0.5 in z IN WORLD
        "push": [0.600, 0.000, 0.50-Z_OFFSET],  # move forward along x by ~15 cm
        "quat": [0.003, 0.717, 0.001, 0.697] # TODO : update this one
    },

    "flashlight": {
        "prep": [0.440, 0.000, 0.25-Z_OFFSET],  # z=0.25 in z IN WORLD
        "push": [0.600, 0.000, 0.25-Z_OFFSET],  # move forward along x by ~15 cm
        "quat": [0.003, 0.717, 0.001, 0.697] # TODO : update this one
    }
}

# PREP_GOAL_XYZ = [0.440, 0.000, 0.215] # z=0.25 in z IN WORLD

# PUSH_GOAL_XYZ = [0.600, 0.000, 0.215]  # move forward along x by ~15 cm

# TF ORI CORRECT = [0.003, 0.717, 0.001, 0.697]
# MUST COMMAND   = [0, 0.7253744, 0, 0.6883546]

def get_object_name():
    """
    Get the object name from ROS parameter server or use default.
    """
    return rospy.get_param("~object_name", None)


def main():
    rospy.init_node("go_forward")

    object_name = get_object_name()
    if object_name is None or object_name not in OBJECT_MOTIONS:
        rospy.logerr(f"[go_forward] Object '{object_name}' not recognized. Set _object_name:= to one of: {list(OBJECT_MOTIONS.keys())}")
        return

    prep_xyz = OBJECT_MOTIONS[object_name]["prep"]
    push_xyz = OBJECT_MOTIONS[object_name]["push"]
    orientation = OBJECT_MOTIONS[object_name]["quat"]

    rospy.loginfo(f"[go_forward] Preparing to push object '{object_name}'.")

    
    # 1) Move to pre-push pose
    # success_prep = execute_motion(
    #     prep_xyz,
    #     cart_speed=0.03,
    #     force_stop=False,
    #     q_desired=orientation,
    #     ft_topic="netft_data_transformed",
    #     arm_logging=False
    # )

    success_prep = go_to_pose_simple(
        prep_xyz,
        # orientation,
        cart_speed=0.03
    )

    if not success_prep:
        rospy.logerr("[go_forward] Pre-push motion failed!")
        return
    else:
        rospy.loginfo("[go_forward] Pre-push pose reached.")

    
    # 2) Execute push motion
    # push_success = execute_motion(
    #     push_xyz,
    #     cart_speed=0.03,
    #     k_safe=0.90,                # stop when force <= 60% of peak
    #     ft_topic="netft_data_transformed",    # change to your topic
    #     contact_thresh=0.4, # Should be higher than sensor noise (when not in contact)
    #     arm_logging=True
    # )

    # if not push_success:
    #     rospy.logerr("[go_forward] Push motion failed!")
    #     return
    # else:
    #     rospy.loginfo("[go_forward] succeeded.")


    # # 3) Return to pre-push pose
    # success_return = execute_motion(
    #     prep_xyz,
    #     cart_speed=0.03,
    #     force_stop=False,
    #     ft_topic="netft_data_transformed",
    #     arm_logging=False
    # )

    # if not success_return:
    #     rospy.logerr("[go_forward] Return motion failed!")
    # else:
    #     rospy.loginfo("[go_forward] Return to initial pose succeeded.")


if __name__ == "__main__":
    main()