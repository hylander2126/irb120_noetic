#!/usr/bin/env python3
import rospy
from com_3d.vel_controller import VelocityController
import numpy as np

# RETRIEVED BY JOGGING ROBOT TO *EXACTLY* ALL ZERO JOINTS ( THESE ARE ALL FROM ROBOT TRANSFORM ON TABLET)
# HOME_XYZ = [0.374000, 0.000000, 0.630000] # TOOL FLANGE
# HOME_QUAT = [0.000000, 0.7071000, 0.000000, 0.7071000]

# RETRIEVED FROM ONSHAPE AND REAL MEASUREMENTS
# Z_OFFSET_MOUNT = 0.035 # Z readings are 35mm higher than expected because of mounting bar
# X_OFFSET_FINGER = 0.081+ 0.114 # X readings are ~192.25 mm 'less' than expected due to FT and finger length

HOME_XYZ = [0.569, 0.0, 0.640] # The hardcorded numbers are the TRUE home pos of FLANGE in TABLE FRAME
HOME_QUAT = [0.0, 0.720, 0.0, 0.7071] # 0.70817, 0.0, 0.70604]

O_OBJ = [0.47065 + 0.081 + 0.114, 0, 0] # Location of object frame in robot (table) frame

# ******** For DIRECT JOINT CONTROL, since sim robot is +35mm from table, joint q is already 'correct' in table frame... *********
### AKA NO NEED TO CORRECT THE Z HEIGHT AT ALL... JUST X

# -----------------------------------------------------------------------------
# PER-OBJECT CARTESIAN GOALS (FINGER TIP COORDINATES wrt ROBOT FRAME)
# -----------------------------------------------------------------------------

OBJECT_MOTIONS = {
    "home": {
        "prep_xyz": HOME_XYZ,
        "prep_quat": HOME_QUAT,
        "prep_q": [0, 0, 0, 0, 0, 0],
    },

    "box": {
        "prep_xyz": [0.5173, 0.0, 0.28],# 0.26], #0.2283], # Corresponds to prep_q below
        "prep_quat": HOME_QUAT,
        # "prep_q": [0, 0.4715, 0.8937, 0, -1.3647, 0],
        # [0, 0.615, 0.867, 0, -1.482, 0], # manual prep joint config ( Yields Z=225mmm..?)
    },
    
    "heart":{
        "prep_xyz": [0.5164, 0.0, 0.170], #0.152], # Corresponds to prep_q below
        "prep_quat": HOME_QUAT,
        # "prep_q": [0, 0.8664, 0.7819, 0, -1.6441, 0]
        # [0, 0.941, 0.748, 0, -1.689, 0],
    },

    # "lshape": {
    #     "prep_xyz": [0.5173, 0.0, 0.27], #0.2283],
    #     "prep_quat": HOME_QUAT,
    #     "prep_q": [0, 1.091, 0.506, 0, -1.597, 0] # manual prep joint config
    # },

    "monitor": {
        "prep_xyz": [0.5667, 0.0, 0.4595], # Corresponds to prep_q below
        "prep_quat": HOME_QUAT,
        # "prep_q": [0, 0.054, 0.552, 0.006, -0.608, 0],
    },

    "flashlight": {
        "prep_xyz": [0.5668, 0.0, 0.1534], # Corresponds to prep_q below
        "prep_quat": HOME_QUAT,
        # "prep_q": [0, 0.978, 0.566, 0, -1.544, 0], # manual prep joint config
    },

    # "soda": {
    #     "prep_xyz": [0.5173, 0.0, 0.27], #0.2283],
    #     "prep_quat": HOME_QUAT,
    #     "prep_q": [0, 0.803, 0.640, 0, -1.445, 0] # manual prep joint config
    # }
}


def main():
    rospy.init_node("vel_move")

    JOINT_TOL   = 2e-3 # 4e-3 # rad

    object_name = rospy.get_param("~object", None)
    if object_name is None or object_name not in OBJECT_MOTIONS:
        rospy.logerr(f"[vel_move] Known Pose '{object_name}' not recognized. Set _object:= to one of: {list(OBJECT_MOTIONS.keys())}")
        return
    else:
        rospy.loginfo(f"[vel_move] Preparing to push object '{object_name}'.")

    ctrl = VelocityController(max_joint_vel=1.0)

    ## =================== BEGIN MOTION SEQUENCE ===================== 
    
    # joint_pos = OBJECT_MOTIONS[object_name]["prep_q"]
    pos = OBJECT_MOTIONS[object_name]["prep_xyz"]
    quat = OBJECT_MOTIONS[object_name]["prep_quat"]

    # success_prep = ctrl.move_to_joint_positions(joint_pos, timeout=8.0, tol=JOINT_TOL)
    success_prep = ctrl.move_to_pose(pos, quat, timeout=8.0, tol=JOINT_TOL)

    if not success_prep:
        rospy.logerr("[vel_move] Pre-push motion failed!")
        return

    rospy.loginfo("[vel_move] Pre-push pose reached.\n")

    rospy.sleep(1.0) # NICE LONG SLEEP TO LET MOTIONS FINISH AND STABILIZE


if __name__ == "__main__":
    main()