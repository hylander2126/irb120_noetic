# velocity_controller_kinematics.py - Custom Kinematics Module

import numpy as np
import tf.transformations as tft
from numpy.linalg import pinv
from scipy.linalg import expm, logm # You'll need scipy for expm/logm

# --- IRB 120 Kinematic Data (Approximate PoE Parameters based on typical 6R) ---

def skew(w):
    """Returns the 3x3 skew-symmetric matrix [w] of a 3x1 vector w."""
    return np.array([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]
    ])


# Six 6-element screw axes (v, w) expressed in the Base Frame at zero position
# These must be derived from the URDF geometry (joint origins and axes).
w1 = [0, 0, 1]
w2 = [0, 1, 0]
w3 = [0, 1, 0]
w4 = [1, 0, 0]
w5 = [0, 1, 0]
w6 = [1, 0, 0]
# v = -[w]p
v1 = np.array([0, 0, 0])
v2 = -skew(w2) @ np.array([0.000, 0.0, 0.290])  # Example origin
v3 = -skew(w3) @ np.array([0.000, 0.0, 0.270])
v4 = -skew(w4) @ np.array([0.000, 0.0, 0.070])
v5 = -skew(w5) @ np.array([0.302, 0.0, 0.000])
v6 = -skew(w6) @ np.array([0.072, 0.0, 0.000])

SCREWS = np.array([
    np.hstack((v1, w1)), np.hstack((v2, w2)), np.hstack((v3, w3)),
    np.hstack((v4, w4)), np.hstack((v5, w5)), np.hstack((v6, w6))
]).T # Transpose to be 6x6 (screw rows)

# Final fixed offset from link_6/flange to tool0: 90 deg about Y
# This is T_Link6_Tool0 at (0, 0, 0)
T_Link6_Tool0_Rot = tft.quaternion_matrix(tft.quaternion_from_euler(0, np.pi/2, 0))[:3, :3]

# M: Zero position transformation matrix (4x4)
M = np.identity(4)
M[:3, 3] = [0.374, 0.0, 0.63]
M[:3, :3] = T_Link6_Tool0_Rot # assuming base to link6 is identity
# M[:3, :3] = tft.quaternion_matrix([0.7071, 0.0, 0.7071, 0.0])[:3, :3] # Example 90 deg rotation about Y
# M[:3, :3] = tft.quaternion_matrix([0, 0.7071, 0, 0.7071])[:3, :3] # Example 90 deg rotation about Y

def screw_to_se3(S):
    """Converts a 6-vector screw S=[v, w] to its 4x4 matrix representation [S]."""
    w = S[3:]
    v = S[:3]
    w_skew = skew(w)
    return np.block([
        [w_skew, v.reshape(3, 1)],
        [np.zeros((1, 4))]
    ])

def forward_kinematics(q):
    """Computes T_EE_Base using PoE."""
    T = M
    for i in range(6):
        S_matrix = screw_to_se3(SCREWS[:, i])
        T = np.dot(expm(S_matrix * q[i]), T)
    return T

def geometric_jacobian(q):
    """
    Computes the 6x6 Geometric Jacobian J_s (space frame) using PoE method.
    The final EE Jacobian J_b is then calculated as J_b = Ad(T_EE_Base^-1) * J_s.
    """
    J_s = np.zeros((6, 6))
    T = np.identity(4)
    for i in range(6):
        # 1. Transform the space-frame screw S_i using the transformation T (T1*T2*...*Ti-1)
        # Adjoint Matrix Ad_T maps the twist from Frame S (base) to Frame T
        # Adjoint matrix is: [ R, 0; skew(p)R, R ]
        
        R = T[:3, :3]
        p = T[:3, 3]
        
        Ad_T = np.block([
            [R, np.zeros((3, 3))],
            [skew(p) @ R, R]
        ])
        
        # 2. The i-th column of the Jacobian is J_s[:, i] = Ad_T @ S_i
        J_s[:, i] = Ad_T @ SCREWS[:, i]
        
        # 3. Update the transformation T
        S_matrix = screw_to_se3(SCREWS[:, i])
        T = np.dot(T, expm(S_matrix * q[i]))

    return J_s