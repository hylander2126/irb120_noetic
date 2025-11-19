# velocity_controller_kinematics.py - Custom Kinematics Module

import numpy as np
from com_3d.helper_fns import *
from numpy.linalg import pinv
from scipy.linalg import expm, logm # You'll need scipy for expm/logm

# --- IRB 120 Kinematic Data (Approximate PoE Parameters based on typical 6R) ---

class Controller:
    def __init__(self):
        self.name = "irb120_controller"

        self.q = np.zeros(6)  # Initial joint angles


        # Six 6-element screw axes (v, w) expressed in the Base Frame at zero position
        w1, w2, w3 = [0, 0, 1], [0, 1, 0], [0, 1, 0]
        w4, w5, w6 = [1, 0, 0], [0, 1, 0], [1, 0, 0]
        
        # joint origins at q=0 in BASE frame
        p1 = np.array([0.0,    0.0,   0.145 ])
        p2 = np.array([0.0,    0.0,   0.290 ])
        p3 = np.array([0.0,    0.0,   0.700 ])
        p4 = np.array([0.0,    0.0,   0.630 ])
        p5 = np.array([0.302,  0.0,   0.630 ])
        p6 = np.array([0.374,  0.0,   0.630 ])

        # Pitches are 0 for all revolute joints
        S1 = ScrewToAxis(w1, p1, 0.0)
        S2 = ScrewToAxis(w2, p2, 0.0)
        S3 = ScrewToAxis(w3, p3, 0.0)
        S4 = ScrewToAxis(w4, p4, 0.0)
        S5 = ScrewToAxis(w5, p5, 0.0)
        S6 = ScrewToAxis(w6, p6, 0.0)

        self.Slist = np.column_stack((S1, S2, S3, S4, S5, S6))  # 6x6 matrix of screws

        # Final fixed offset from link_6/flange to tool0: 90 deg about Y
        T_Link6_Tool0_Rot = np.array([[0, 0, 1],
                                      [0, 1, 0],
                                      [-1, 0, 0]])

        # M: Zero position transformation matrix (4x4)
        self.M = np.identity(4)
        self.M[:3, 3] = [0.374, 0.0, 0.63]
        self.M[:3, :3] = T_Link6_Tool0_Rot # assuming base to link6 is identity

    def screw_to_se3(self, S):
        """Converts a 6-vector screw S=[v, w] to its 4x4 matrix representation [S]."""
        w = S[3:]
        v = S[:3]
        w_skew = VecToso3(w)
        return np.block([
            [w_skew, v.reshape(3, 1)],
            [np.zeros((1, 4))]
        ])

    def forward_kinematics(self, q):
        """Computes T_EE_Base using PoE."""
        T = self.M
        for i in range(6):
            S_matrix = self.screw_to_se3(self.Slist[:, i])
            T = np.dot(expm(S_matrix * q[i]), T)
        return T
    
    def ikin_space(self, T, theta_list, eomg, ev):
        """IKin in space frame
        theta_list: desired end-effector pose as 4x4 matrix
        eomg: orientation error tolerance, small positive number
        ev: position error tolerance, small positive number
        """

        # TODO this is broken...
        q = IKinSpace(self.Slist, self.M, T, theta_list, eomg, ev)
        return q
        

    def geometric_jacobian(self, q):
        q = np.asarray(q).reshape(6,)
        return JacobianSpace(self.Slist, q)


def main():
    controller = Controller()
    # q_test = np.ones(6) * np.pi/6
    # q_test = np.zeros(6)
    q_test = [0, 0.1, 0, 0, 0, 0]
    # q_test = [0, 0.615, 0.867, 0, -1.482, 0] # Known to yield xyz~[0.369, 0, 0.215]
    T_ee = controller.forward_kinematics(q_test)
    T_ee_mr = FKinSpace(controller.M, controller.Slist.T, q_test)
    q_des = controller.ikin_space(T_ee, np.ones(6)*0.2, 0.001, 0.001)
    J_geo = controller.geometric_jacobian(q_test)

    print("End-Effector Transformation T_EE_Base:\n", np.round(T_ee, 3))
    print("End-Effector Transformation from FKinSpace T_EE_Base:\n", np.round(T_ee_mr, 3))
    print("Inverse Kinematics Joint Angles q_des:\n", q_des[0])#np.round(q_des, 3))
    print("Geometric Jacobian J_geo:\n", np.round(J_geo, 3))

if __name__ == "__main__":
    main()