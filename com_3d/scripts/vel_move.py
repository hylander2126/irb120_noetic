#!/usr/bin/env python3
import rospy
from com_3d.vel_controller import VelocityController
import numpy as np

# RETRIEVED BY JOGGING ROBOT TO *EXACTLY* ALL ZERO JOINTS
HOME_XYZ = [0.374000, 0.000000, 0.630000] # TOOL FLANGE IN ROBOT FRAME (from robot, not TF) (technically 374.01, 0.01, 629.99 mm)
HOME_QUAT = [0.000000, 0.7071000, 0.000000, 0.7071000]


# RETRIEVED FROM ONSHAPE AND REAL MEASUREMENTS
Z_OFFSET_MOUNT = 0.035 # Z readings are 35mm higher than expected because of mounting bar
X_OFFSET_FINGER = 0.08225+ 0.11 # X readings are ~192.25 mm 'less' than expected due to FT and finger length

# ******** For DIRECT JOINT CONTROL, since sim robot is +35mm from table, joint q is already 'correct' in table frame... *********

DEFAULT_OBJECT = "box"

# -----------------------------------------------------------------------------
# PER-OBJECT CARTESIAN GOALS (FINGER TIP COORDINATES wrt ROBOT FRAME)
# -----------------------------------------------------------------------------

OBJECT_MOTIONS = {
    "box": {
        "prep": [HOME_XYZ[0]-0.05, HOME_XYZ[1], 0.25-Z_OFFSET_MOUNT], # Z=0.25 in TABLE FRAME
        "push": [HOME_XYZ[0]-0.05+0.2, HOME_XYZ[1], 0.25-Z_OFFSET_MOUNT], # Forward along x by 20cm (note how this is )
        # "quat": [0.003, 0.717, 0.001, 0.697],
        "prep_q": [0, 0.615, 0.867, 0, -1.482, 0] # manual prep joint config ( Yields Z=225mmm..?)
    },
    
    "heart":{
        "prep": [HOME_XYZ[0]-0.05, HOME_XYZ[1], 0.175-Z_OFFSET_MOUNT], # Z=0.175 in TABLE FRAME
        "push": [HOME_XYZ[0]-0.05+0.2, HOME_XYZ[1], 0.175-Z_OFFSET_MOUNT], # Forward along x by 20cm (note how this is )
        # "quat": [0.003, 0.717, 0.001, 0.697], # TODO: update this one
        "prep_q": [0, 0.941, 0.748, 0, -1.689, 0] # manual prep joint config
    },

    "lshape": {
        "prep": [HOME_XYZ[0], HOME_XYZ[1], 0.145-Z_OFFSET_MOUNT],  # Z=0.145 in TABLE FRAME
        "push": [HOME_XYZ[0]-0.05+0.2, HOME_XYZ[1], 0.145-Z_OFFSET_MOUNT], # Forward along x by 20cm (note how this is )
        # "quat": [-0.001, 0.703, 0.005, 0.711], # TODO : update this one
        "prep_q": [0, 1.091, 0.506, 0, -1.597, 0] # manual prep joint config
    },

    # "monitor": {
    #     "prep": [0.440, 0.000, 0.50],  # z=0.5 in z IN WORLD
    #     "push": [0.600, 0.000, 0.50],  # move forward along x by ~15 cm
    #     # "quat": [0.003, 0.717, 0.001, 0.697], # TODO : update this one
    #     # "prep_q": [-0.004, 1.154, 0.437, -0.008, -1.566, 0.001] # manual prep joint config
    # },

    "flashlight": {
        "prep": [HOME_XYZ[0], HOME_XYZ[1], 0.175-Z_OFFSET_MOUNT],  # z=0.25 in z IN WORLD
        "push": [HOME_XYZ[0]-0.05+0.2, HOME_XYZ[1], 0.175-Z_OFFSET_MOUNT], # Forward along x by 20cm (note how this is )
        # "quat": [0.003, 0.717, 0.001, 0.697], # TODO : update this one
        "prep_q": [0, 0.978, 0.566, 0, -1.544, 0] # manual prep joint config
    }
}

def get_object_name():
    """
    Get the object name from ROS parameter server or use default.
    """
    return rospy.get_param("~object", None)


def main():
    rospy.init_node("go_forward")

    object_name = get_object_name()
    if object_name is None or object_name not in OBJECT_MOTIONS:
        rospy.logerr(f"[go_forward] Object '{object_name}' not recognized. Set _object:= to one of: {list(OBJECT_MOTIONS.keys())}")
        return

    ctrl = VelocityController(max_joint_vel=1.0)

    # prep_xyz = OBJECT_MOTIONS[object_name]["prep"]
    # push_xyz = OBJECT_MOTIONS[object_name]["push"]
    # orientation = OBJECT_MOTIONS[object_name]["quat"]
    joint_pos = OBJECT_MOTIONS[object_name]["prep_q"]

    rospy.loginfo(f"[go_forward] Preparing to push object '{object_name}'.")

    
    # 1) Move to pre-push pose
    success_prep = ctrl.move_to_joint_positions(
        joint_pos, timeout=5.0, tol=7e-3
    )
    if not success_prep:
        rospy.logerr("[go_forward] Pre-push motion failed!")
        return
    else:
        rospy.loginfo("[go_forward] Pre-push pose reached.\n")

    
    # 2) Execute push motion
    success_push = ctrl.cartesian_velocity(
        v=[0.01, 0, 0], # XYZ
        w=[0, 0, 0],   # RPY
        duration=6.0,
        k_safe=0.5
    )

    if not success_push:
        rospy.logerr("[go_forward] Push motion failed!")
        return
    else:
        rospy.loginfo("[go_forward] succeeded.\n")


    # 3) Return to pre-push pose
    success_return = ctrl.move_to_joint_positions(
        joint_pos, timeout=5.0
    )

    if not success_return:
        rospy.logerr("[go_forward] Return motion failed!")
    else:
        rospy.loginfo("[go_forward] Return to initial pose succeeded.\n")


if __name__ == "__main__":
    main()