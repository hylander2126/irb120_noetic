# velocity_controller_kinematics.py - Custom Kinematics Module

import numpy as np
try:
    from com_3d.helper_fns import *
except ImportError:
    from helper_fns import *
from numpy.linalg import pinv
from scipy.linalg import expm, logm # You'll need scipy for expm/logm

# --- IRB 120 Kinematic Data (Approximate PoE Parameters based on typical 6R) ---

class VelController:
    def __init__(self):
        self.name = "irb120_controller"

        self.q = np.zeros(6)  # Initial joint angles

        # ====================== SPACE FRAME SCREW AXES =========================
        # Six 6-element screw axes (v, w) expressed in the Base Frame at zero position
        w1, w2, w3 = [0, 0, 1], [0, -1, 0], [0, -1, 0]
        w4, w5, w6 = [1, 0, 0], [0, -1, 0], [1, 0, 0]

        # joint origins at q=0 in BASE frame
        p1 =      np.array([0.000,  0,   0.145 ]) # 0.145
        p2 = p1 + np.array([0.000,  0,   0.145 ])
        p3 = p2 + np.array([0.000,  0,   0.270 ])
        p4 = p3 + np.array([0.134,  0,   0.070 ])
        p5 = p4 + np.array([0.168,  0,   0.000 ])
        p6 = p5 + np.array([0.072,  0,   0.000 ])

        v1 = -VecToso3(w1) @ p1
        v2 = -VecToso3(w2) @ p2
        v3 = -VecToso3(w3) @ p3
        v4 = -VecToso3(w4) @ p4
        v5 = -VecToso3(w5) @ p5
        v6 = -VecToso3(w6) @ p6

        S1 = np.concatenate((w1, v1))
        S2 = np.concatenate((w2, v2))
        S3 = np.concatenate((w3, v3))
        S4 = np.concatenate((w4, v4))
        S5 = np.concatenate((w5, v5))
        S6 = np.concatenate((w6, v6))

        self.Sspace = np.column_stack((S1, S2, S3, S4, S5, S6))  # 6x6 matrix of screws

        # ====================== BODY FRAME SCREW AXES =========================
        # TODO: Add this if space frame doesn't work


        # Final fixed offset from link_6/flange to tool0: 90 deg about Y
        T_Link6_Tool0_Rot = np.array([[0, 0, 1],
                                      [0, 1, 0],
                                      [-1, 0, 0]])
        

        R_l6_tool0 = np.array([[0,  0, 1],
                               [0, -1, 0],
                               [1,  0, 0]])
        # Equivalent!^^^ vvv
        R_base_l6 = np.array([[0, 0, 1],
                              [0, -1, 0],
                              [1, 0, 0]])

        # M: Zero position transformation matrix (4x4)
        self.M = np.identity(4)
        self.M[:3, 3] = p6
        self.M[:3, :3] = R_l6_tool0 # assuming base to link6 is identity
        # self.M[:3, :3] = T_Link6_Tool0_Rot

    def FK_space(self, q):
        """Computes T_EE_Base using PoE."""
        T = self.M
        for i in range(6):
            S_matrix = screw_to_se3(self.Sspace[:, i])
            T = np.dot(expm(S_matrix * q[i]), T)
        return T
    
    def IK_space(self, T, theta_list, eomg, ev):
        """IKin in space frame
        theta_list: desired end-effector pose as 4x4 matrix
        eomg: orientation error tolerance, small positive number
        ev: position error tolerance, small positive number
        """

        # TODO this is broken...
        q = IKinSpace(self.Sspace, self.M, T, theta_list, eomg, ev)
        return q
        
    def J_geometric(self, q):
        """Jacobian in form w,v (space frame)"""
        q = np.asarray(q).reshape(6,)
        return JacobianSpace(self.Sspace, q)
    
    def J_body(self, q):
        """Jacobian in form w,v (body frame)"""
        q = np.asarray(q).reshape(6,)
        return JacobianBody(self.Sspace, self.M, q)
    
    def manipulability(self, J):
        """Computes the Yoshikawa manipulability measure."""
        JJt = J @ J.T

        # Numerical safety: determinant can be slightly negative due to numerical errors
        det_val = np.linalg.det(JJt)
        det_val = max(det_val, 0)
        return float(np.sqrt(det_val))


    def damped_pinv(self, J, lam):
        """
        Damped least-squares pseudoinverse:
        J^T (J J^T + lambda^2 I)^{-1}
        """
        JJt = J @ J.T
        return J.T @ np.linalg.inv(JJt + (lam ** 2) * np.eye(J.shape[0]))


def main():
    controller = VelController()
    # q_test = np.ones(6) * np.pi/6
    q_test = np.zeros(6)
    # q_test = [0, 0.1, 0, 0, 0, 0]
    # q_test = [0, 0.615, 0.867, 0, -1.482, 0] # Known to yield xyz~[0.369, 0, 0.215]
    T_ee = controller.FK_space(q_test)
    J_geo = controller.J_geometric(q_test)

    print("End-Effector Transformation T_EE_Base:\n", np.round(T_ee, 3))
    from scipy.spatial.transform import Rotation as R
    r = R.from_matrix(T_ee[:3, :3])
    print("Euler angles for this transformation (deg):\n", np.round(np.degrees(r.as_euler('xyz')), 2))
    # print("Geometric Jacobian J_geo:\n", np.round(J_geo, 3))

    # Test cartesian (space-frame) velocity (what are q_dot for rpy,xyz vel?)
    v_test = np.array([0.1, 0, 0, 0, 0, 0])  # Move down in Z at 0.1 m/s
    q_dot = pinv(J_geo) @ v_test
    print("\nFor desired cartesian vel", v_test)
    print("Required joint velocities q_dot:\n", np.round(q_dot, 4))

if __name__ == "__main__":
    main()