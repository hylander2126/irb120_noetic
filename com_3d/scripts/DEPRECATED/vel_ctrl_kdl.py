#!/usr/bin/env python3
import numpy as np

import rospy
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL as kdl


class KDLRobot:
    def __init__(
        self,
        base_link="base_link",
        ee_link="tool0",
        urdf_param="robot_description",
        damping=1e-2,
    ):
        """
        KDL-based kinematics backend.

        base_link: name of the base link in URDF
        ee_link:   name of the tool/EEF link in URDF (tool0 in your xacro)
        """
        self.base_link = base_link
        self.ee_link = ee_link

        # Load URDF from parameter server
        rospy.loginfo(f"[KDLRobot] Loading URDF from param '{urdf_param}'...")
        robot = URDF.from_parameter_server(key=urdf_param)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("[KDLRobot] Failed to build KDL tree from URDF.")

        self.chain = tree.getChain(self.base_link, self.ee_link)
        self.nj = self.chain.getNrOfJoints()
        rospy.loginfo(f"[KDLRobot] KDL chain from {self.base_link} to "
                      f"{self.ee_link}, joints: {self.nj}")

        # Solvers
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

        # Velocity IK solver (weighted / damped least squares if available)
        try:
            self.ik_solver_vel = kdl.ChainIkSolverVel_wdls(self.chain)
            # Damping (lambda) to make it robust near singularities
            self.ik_solver_vel.setLambda(damping)
            rospy.loginfo(f"[KDLRobot] Using ChainIkSolverVel_wdls, damping={damping}")
        except AttributeError:
            # Fallback: plain pseudoinverse velocity IK
            self.ik_solver_vel = kdl.ChainIkSolverVel_pinv(self.chain)
            rospy.logwarn(
                "[KDLRobot] ChainIkSolverVel_wdls not available. "
                "Using ChainIkSolverVel_pinv (less robust near singularities)."
            )

    # -------------------- Helpers --------------------

    def _to_kdl_jnt_array(self, q_np):
        """Convert numpy joint vector to KDL JntArray."""
        q = kdl.JntArray(self.nj)
        for i in range(self.nj):
            q[i] = float(q_np[i])
        return q

    def _jacobian_to_numpy(self, jac_kdl):
        """Convert KDL Jacobian -> 6 x N numpy array."""
        J = np.zeros((6, self.nj))
        for i in range(6):
            for j in range(self.nj):
                J[i, j] = jac_kdl[i, j]
        return J

    # -------------------- Public API --------------------

    def jacobian(self, q_np):
        """
        Compute space-frame Jacobian J_s(q) as 6 x N numpy array.
        """
        q_kdl = self._to_kdl_jnt_array(q_np)
        jac_kdl = kdl.Jacobian(self.nj)
        self.jac_solver.JntToJac(q_kdl, jac_kdl)
        return self._jacobian_to_numpy(jac_kdl)

    def manipulability(self, q_np):
        """
        Yoshikawa manipulability index mu = sqrt(det(J J^T)).
        """
        J = self.jacobian(q_np)
        JJt = J @ J.T
        det_val = np.linalg.det(JJt)
        det_val = max(det_val, 0.0)  # numerical safety
        return float(np.sqrt(det_val))

    def ik_velocity(self, q_np, twist_np):
        """
        KDL velocity IK: given current joints q_np and desired twist in BASE frame,
        return joint velocities dq.

        twist_np: [vx, vy, vz, wx, wy, wz] in base frame.
        """
        assert twist_np.shape == (6,)
        q_kdl = self._to_kdl_jnt_array(q_np)

        v = kdl.Vector(twist_np[0], twist_np[1], twist_np[2])
        w = kdl.Vector(twist_np[3], twist_np[4], twist_np[5])
        twist_kdl = kdl.Twist(v, w)

        dq_kdl = kdl.JntArray(self.nj)
        ret = self.ik_solver_vel.CartToJnt(q_kdl, twist_kdl, dq_kdl)
        if ret < 0:
            # KDL usually returns 0 on success, negative on error
            raise RuntimeError(f"[KDLRobot] IK velocity solver failed, code={ret}")

        dq = np.zeros(self.nj)
        for i in range(self.nj):
            dq[i] = dq_kdl[i]
        return dq
    
def main():
    rospy.init_node("kdl_robot_test")

    robot = KDLRobot()

    q_test = np.array([0.0, 0.5, 0.5, 0.0, -1.0, 0.0])
    J = robot.jacobian(q_test)
    mu = robot.manipulability(q_test)

    rospy.loginfo(f"Jacobian at q={q_test}:\n{J}")
    rospy.loginfo(f"Manipulability at q={q_test}: mu={mu}")

    twist_test = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.1])  # desired EE twist
    dq = robot.ik_velocity(q_test, twist_test)
    rospy.loginfo(f"Joint velocities for twist {twist_test}:\n{dq}")


if __name__ == "__main__":
    main()