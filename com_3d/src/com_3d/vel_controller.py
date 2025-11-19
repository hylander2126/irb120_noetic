#!/usr/bin/env python3
import math
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from threading import Lock
from abb_robot_msgs.srv import TriggerWithResultCode


from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL as kdl


# Global EGM/Logging wrappers
EGM_START_SRV = "/rws/sm_addin/start_egm_joint"
EGM_STOP_SRV  = "/rws/sm_addin/stop_egm"
def _start_egm():
    rospy.wait_for_service(EGM_START_SRV)
    try:
        rospy.ServiceProxy(EGM_START_SRV, TriggerWithResultCode)(TriggerWithResultCode._request_class())
    except Exception as e:
        rospy.logwarn(f"start_egm call failed: {e}")

def _stop_egm():
    rospy.wait_for_service(EGM_STOP_SRV)
    try:
        rospy.ServiceProxy(EGM_STOP_SRV, TriggerWithResultCode)(TriggerWithResultCode._request_class())
    except Exception as e:
        rospy.logwarn(f"stop_egm call failed: {e}")


class VelocityController:
    """
    High-level controller that exposes just two clean methods:

        move_to_joint_positions(q_target, timeout=5.0)
        cartesian_velocity(v, w, duration)

    where:
      - q_target is a list of joint angles (rad) in this controller's joint order
      - v = [vx, vy, vz] in BASE frame (m/s)
      - w = [wx, wy, wz] in BASE frame (rad/s)

    Internals (KDL, URDF, Jacobians, etc.) are hidden.
    """

    def __init__(
        self,
        base_link="base_link",
        tip_link="tool0",
        urdf_param="robot_description",
        joint_state_topic="/egm/joint_states",
        vel_cmd_topic="/egm/joint_group_velocity_controller/command",
        rate_hz=100.0,
        max_joint_vel=2.0,          # rad/s safety clamp
        joint_safety_margin=0.05,   # rad away from hard limits
    ):
        self.base_link = base_link
        self.tip_link = tip_link
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz
        self.max_joint_vel = float(max_joint_vel)
        self.joint_safety_margin = float(joint_safety_margin)

        # ---------------------------------------------------------------------
        # 1) Parse URDF and build KDL chain
        # ---------------------------------------------------------------------
        rospy.loginfo(f"[VC] Loading URDF from '{urdf_param}'...")
        robot = URDF.from_parameter_server(key=urdf_param)

        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("[VC] Failed to parse URDF into KDL tree")

        self.chain = tree.getChain(self.base_link, self.tip_link)
        self.nj = self.chain.getNrOfJoints()

        # Extract joint order from the chain
        self.joint_names = []
        for i in range(self.chain.getNrOfSegments()):
            seg = self.chain.getSegment(i)
            jnt = seg.getJoint()
            # "None" joints are fixed; skip those
            jtype_name = jnt.getTypeName()
            # if (jnt.getType() != kdl.Joint.Fixed):
            if jtype_name != "None":
                self.joint_names.append(jnt.getName())

        rospy.loginfo(f"[VC] KDL chain {self.base_link} -> {self.tip_link} "
                      f"with joints: {self.joint_names}")

        # Joint limits from URDF (aligned with self.joint_names)
        lower, upper = [], []
        for name in self.joint_names:
            j = robot.joint_map[name]
            if j.limit is None:
                lower.append(-math.inf)
                upper.append(+math.inf)
            else:
                lower.append(j.limit.lower)
                upper.append(j.limit.upper)
        self.joint_lower = np.array(lower)
        self.joint_upper = np.array(upper)

        # ---------------------------------------------------------------------
        # 2) KDL solvers
        # ---------------------------------------------------------------------
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

        # Damped (weighted) velocity IK if available, else pseudoinverse
        try:
            self.ik_solver_vel = kdl.ChainIkSolverVel_wdls(self.chain)
            self.ik_solver_vel.setLambda(1e-2)  # damping factor
            rospy.loginfo("[VC] Using ChainIkSolverVel_wdls (damped IK).")
        except AttributeError:
            self.ik_solver_vel = kdl.ChainIkSolverVel_pinv(self.chain)
            rospy.logwarn("[VC] ChainIkSolverVel_wdls not available; using "
                          "ChainIkSolverVel_pinv (less robust near singularities).")
            
        # ---- NEW: joint-limited position IK (NR_JL) ----
        q_min = kdl.JntArray(self.nj)
        q_max = kdl.JntArray(self.nj)
        for i in range(self.nj):
            # fall back to huge limits if URDF has +/-inf (shouldn't, but just in case)
            lo = self.joint_lower[i]
            hi = self.joint_upper[i]
            q_min[i] = lo if np.isfinite(lo) else -1e9
            q_max[i] = hi if np.isfinite(hi) else  1e9

        self.ik_solver_pos = kdl.ChainIkSolverPos_NR_JL(
            self.chain,
            q_min,
            q_max,
            self.fk_solver,
            self.ik_solver_vel,
            100,        # max iterations
            1e-4        # eps
        )
        rospy.loginfo("[VC] Using ChainIkSolverPos_NR_JL for position IK with joint limits.")

        # ---------------------------------------------------------------------
        # 3) ROS I/O
        # ---------------------------------------------------------------------
        self._state_lock = Lock()
        self._q = None  # np.array, aligned with self.joint_names

        self.joint_sub = rospy.Subscriber(
            joint_state_topic, JointState, self._joint_state_cb, queue_size=1
        )
        self.vel_pub = rospy.Publisher(
            vel_cmd_topic, Float64MultiArray, queue_size=1
        )

        self.rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("[VC] Waiting for first joint state...")
        self._wait_for_joint_state()
        rospy.loginfo("[VC] Ready.")

    # =====================================================================
    #   PUBLIC API
    # =====================================================================

    def move_to_joint_positions(self, q_target, timeout=5.0, kp=2.0, tol=1e-3):
        """
        Drive joints to q_target using a simple velocity P-controller.

        q_target: list/np.array of target joint angles (rad),
                  same order as self.joint_names.
        timeout:  seconds before giving up.
        kp:       proportional gain [rad/s per rad error].
        tol:      norm of joint error to consider 'done'.
        """
        q_target = np.asarray(q_target, dtype=float)
        if q_target.shape != (self.nj,):
            raise ValueError(
                f"q_target must be length {self.nj} (got {q_target.shape})\n"
                f"Joint order is: {self.joint_names}"
            )

        _start_egm()
        rospy.sleep(0.5)

        start_time = rospy.Time.now()
        rospy.loginfo(f"[VC] move_to_joint_positions -> {q_target}")

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("[VC] move_to_joint_positions timed out.")
                break

            with self._state_lock:
                q_curr = None if self._q is None else self._q.copy()
            if q_curr is None:
                self.rate.sleep()
                continue

            err = q_target - q_curr
            if np.linalg.norm(err) < tol:
                rospy.loginfo("[VC] Joint target reached.")
                break

            q_dot_cmd = kp * err

            # TEMP Let's see what joint commands are being calced
            rospy.loginfo_throttle(1.0, f"[VC] q_dot_cmd: {np.round(q_dot_cmd, 3)}")

            # Joint-limit avoidance (very simple clamp near limits)
            q_dot_cmd = self._apply_joint_limit_avoidance(q_curr, q_dot_cmd)

            # Saturate joint velocities
            q_dot_cmd = np.clip(q_dot_cmd, -self.max_joint_vel, self.max_joint_vel)

            self._publish_velocity(q_dot_cmd)
            self.rate.sleep()

        # Stop motion
        self._publish_velocity(np.zeros(self.nj))

        rospy.sleep(0.5)
        _stop_egm()
        return True

    def cartesian_velocity(self, v, w, duration):
        """
        Apply a constant Cartesian twist for 'duration' seconds.

        v: [vx, vy, vz] in BASE frame (m/s)
        w: [wx, wy, wz] in BASE frame (rad/s)
        duration: seconds
        """
        twist_np = np.asarray(list(v) + list(w), dtype=float)
        if twist_np.shape != (6,):
            raise ValueError("v and w must each be length 3")

        rospy.loginfo(f"[VC] cartesian_velocity v={v}, w={w}, duration={duration}")

        _start_egm()
        rospy.sleep(0.5)

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > duration:
                break

            with self._state_lock:
                q_curr = None if self._q is None else self._q.copy()
            if q_curr is None:
                self.rate.sleep()
                continue

            # Compute joint velocities via KDL velocity IK
            q_dot_cmd = self._ik_velocity(q_curr, twist_np)

            # TEMP Let's see what joint commands are being calced
            rospy.loginfo_throttle(1.0, f"[VC] q_dot_cmd: {np.round(q_dot_cmd, 3)}")

            # Joint-limit avoidance
            q_dot_cmd = self._apply_joint_limit_avoidance(q_curr, q_dot_cmd)

            # Clamp velocities
            q_dot_cmd = np.clip(q_dot_cmd, -self.max_joint_vel, self.max_joint_vel)

            self._publish_velocity(q_dot_cmd)
            self.rate.sleep()

        # Stop after duration
        self._publish_velocity(np.zeros(self.nj))

        rospy.sleep(0.5)
        _stop_egm()
        return True
    

    def compute_ik(self, xyz, rpy, q_seed=None):
        """
        Public wrapper to get joint angles for a desired pose (xyz, rpy).

        xyz: [x, y, z] in BASE frame (meters)
        rpy: [roll, pitch, yaw] in BASE frame (rad)
        q_seed: optional initial guess array.

        Returns: np.array length nj (joint angles).
        """
        xyz = np.asarray(xyz, dtype=float)
        rpy = np.asarray(rpy, dtype=float)
        if xyz.shape != (3,) or rpy.shape != (3,):
            raise ValueError("xyz and rpy must each be length 3")

        q_sol = self._ik_position(xyz, rpy, q_seed=q_seed)
        rospy.loginfo(f"[VC] compute_ik -> {q_sol}")
        return q_sol
    

    def move_to_pose(self, xyz, rpy, timeout=5.0, kp=2.0, tol=1e-3, q_seed=None):
        """
        Solve IK for (xyz, rpy) and then move there using move_to_joint_positions.

        xyz, rpy: pose in BASE frame.
        """
        q_target = self.compute_ik(xyz, rpy, q_seed=q_seed)
        return self.move_to_joint_positions(
            q_target=q_target,
            timeout=timeout,
            kp=kp,
            tol=tol,
        )


    # =====================================================================
    #   INTERNAL HELPERS
    # =====================================================================

    def _joint_state_cb(self, msg):
        # Build q aligned with self.joint_names
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        q = np.zeros(self.nj)
        for i, jname in enumerate(self.joint_names):
            if jname not in name_to_idx:
                # If a joint is missing, leave it as previous or zero.
                continue
            q[i] = msg.position[name_to_idx[jname]]

        with self._state_lock:
            self._q = q

    def _wait_for_joint_state(self, timeout=5.0):
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            with self._state_lock:
                if self._q is not None:
                    return
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn("[VC] No joint state received yet (still waiting).")
                t0 = rospy.Time.now()
            self.rate.sleep()

    def _to_kdl_jnt_array(self, q_np):
        qa = kdl.JntArray(self.nj)
        for i in range(self.nj):
            qa[i] = float(q_np[i])
        return qa

    def _ik_velocity(self, q_np, twist_np):
        """Use KDL's velocity IK solver to map twist -> q_dot."""
        q_kdl = self._to_kdl_jnt_array(q_np)

        v = kdl.Vector(twist_np[0], twist_np[1], twist_np[2])
        w = kdl.Vector(twist_np[3], twist_np[4], twist_np[5])
        twist_kdl = kdl.Twist(v, w)

        dq_kdl = kdl.JntArray(self.nj)
        ret = self.ik_solver_vel.CartToJnt(q_kdl, twist_kdl, dq_kdl)
        if ret < 0:
            rospy.logwarn_throttle(
                1.0, f"[VC] IK velocity solver failed with code={ret}; sending zero vel"
            )
            return np.zeros(self.nj)

        dq = np.zeros(self.nj)
        for i in range(self.nj):
            dq[i] = dq_kdl[i]
        return dq

    def _apply_joint_limit_avoidance(self, q_curr, q_dot_cmd):
        """
        Very simple joint-limit avoidance:
        - Predict next position
        - If we are moving further out of the safe region, zero that joint's velocity.
        """
        q_next = q_curr + q_dot_cmd * self.dt

        lower_safe = self.joint_lower + self.joint_safety_margin
        upper_safe = self.joint_upper - self.joint_safety_margin

        for i in range(self.nj):
            # Lower side
            if np.isfinite(lower_safe[i]) and q_next[i] < lower_safe[i] and q_dot_cmd[i] < 0.0:
                q_dot_cmd[i] = 0.0
                rospy.logwarn_throttle(
                    1.0, f"[VC] Joint {self.joint_names[i]} near LOWER limit; zeroing vel."
                )
            # Upper side
            if np.isfinite(upper_safe[i]) and q_next[i] > upper_safe[i] and q_dot_cmd[i] > 0.0:
                q_dot_cmd[i] = 0.0
                rospy.logwarn_throttle(
                    1.0, f"[VC] Joint {self.joint_names[i]} near UPPER limit; zeroing vel."
                )

        return q_dot_cmd


    def _publish_velocity(self, q_dot_cmd):
        msg = Float64MultiArray()
        msg.data = q_dot_cmd.tolist()
        self.vel_pub.publish(msg)


    def _frame_from_xyz_rpy(self, xyz, rpy):
        """
        Build a KDL.Frame from position + RPY (in radians), all in BASE frame.

        xyz: [x, y, z]
        rpy: [roll, pitch, yaw]
        """
        x, y, z = xyz
        rr, rp, ry = rpy

        R = kdl.Rotation.RPY(rr, rp, ry)
        p = kdl.Vector(x, y, z)
        return kdl.Frame(R, p)


    def _ik_position(self, xyz, rpy, q_seed=None):
        """
        Solve joint positions for a desired pose (xyz, rpy) using joint-limited IK.

        xyz, rpy in BASE frame.
        q_seed: optional initial guess (np.array length nj). If None, use current q.
        Returns: np.array length nj (joint angles) or raises RuntimeError on failure.
        """
        F_des = self._frame_from_xyz_rpy(xyz, rpy)

        # Seed
        if q_seed is None:
            with self._state_lock:
                q_curr = None if self._q is None else self._q.copy()
            if q_curr is None:
                raise RuntimeError("[VC] No joint state available for IK seed")
            q_seed_np = q_curr
        else:
            q_seed_np = np.asarray(q_seed, dtype=float)
            if q_seed_np.shape != (self.nj,):
                raise ValueError(f"q_seed must be length {self.nj}")
        
        q_seed_kdl = kdl.JntArray(self.nj)
        for i in range(self.nj):
            q_seed_kdl[i] = q_seed_np[i]

        q_sol_kdl = kdl.JntArray(self.nj)
        ret = self.ik_solver_pos.CartToJnt(q_seed_kdl, F_des, q_sol_kdl)
        if ret < 0:
            raise RuntimeError(f"[VC] Position IK failed with code={ret}")

        q_sol = np.zeros(self.nj)
        for i in range(self.nj):
            q_sol[i] = q_sol_kdl[i]
        return q_sol
