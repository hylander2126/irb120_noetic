#!/usr/bin/env python3
import math
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Empty, Bool
from geometry_msgs.msg import PoseStamped

from threading import Lock
from abb_robot_msgs.srv import TriggerWithResultCode
from abb_egm_msgs.msg import EGMState

from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL as kdl

EGM_START_SRV = "/rws/sm_addin/start_egm_joint"
EGM_STOP_SRV  = "/rws/sm_addin/stop_egm"


def _wait_for_egm_active(target_active=True, timeout=5.0):
    """Blocks until EGM state matches target_active."""
    t_start = rospy.Time.now()
    while (rospy.Time.now() - t_start).to_sec() < timeout and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message("/egm/egm_states", EGMState, timeout=0.1)
            if msg.egm_channels: # If the message isn't empty
                # Issue command if state doesn't match
                if msg.egm_channels[0].active != target_active:
                    _call_egm_service(EGM_START_SRV if target_active else EGM_STOP_SRV)
                else:
                    return True # Desired state reached (or was already correct)
        except rospy.ROSException:
            continue # Timeout or shutdown occurred while waiting for the message
    raise RuntimeError(f"EGM did not reach active={target_active} within timeout.")


def _call_egm_service(service_name):
    """Generic safe service caller for EGM commands."""
    try:
        rospy.wait_for_service(service_name, timeout=2.0)
        rospy.ServiceProxy(service_name, TriggerWithResultCode)(TriggerWithResultCode._request_class()) # second parentheses calls the service
    except (rospy.ServiceException, rospy.ROSInterruptException, rospy.ROSException) as e:
        rospy.logwarn(f"[VC] EGM Service call failed ({service_name}): {e}")


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
        base_link           = "table",
        tip_link            = "finger_tip",
        urdf_param          = "robot_description",
        joint_state_topic   = "/egm/joint_states",
        vel_cmd_topic       = "/egm/joint_group_velocity_controller/command",
        rate_hz             = 100.0,
        max_joint_vel       = 1.57,   # 0.5*pi rad/s safety clamp
        joint_safety_margin = 0.05,   # rad away from hard limits
        floor_z_margin      = 0.06,   # Lowest the fingertip should be allowed to go
    ):
        self.base_link           = base_link
        self.tip_link            = tip_link
        self.rate_hz             = float(rate_hz)
        self.dt                  = 1.0 / self.rate_hz
        self.max_joint_vel       = float(max_joint_vel)
        self.joint_safety_margin = float(joint_safety_margin)
        self.floor_z_margin      = float(floor_z_margin)

        # FK state tracking
        self.ee_max_age                 = 0.1 # 100 ms --> used to be 0.02 # 20 ms but too strict
        self.last_cartesian_duration    = None

        # Joint goal gains
        self.kp_joints = np.array([4, 4, 4, 4, 6, 4], dtype=float)#4.0  # [rad/s per rad error]
        self.ki_joints = np.array([0.4, 0.4, 0.4, 0.4, 0.8, 0.4], dtype=float)  # [rad/s^2 per rad error]
        self.kd_joints = np.array([0.1, 0.1, 0.1, 0.1, 0.2, 0.1], dtype=float) # [rad/s per rad error rate]
        self.I_max    = np.full(6, 0.5, dtype=float)  # rad*sec

        # Cartesian gains
        self.kp_orient = 1.0  # [rad/s per rad error]
        self.kp_zlock  = 1.0  # [m/s per m error]


        # 1) Parse URDF and build KDL chain
        # ---------------------------------------------------------------------
        rospy.loginfo(f"[VC] Loading URDF from '{urdf_param}'...")
        
        robot = URDF.from_parameter_server(key=urdf_param)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("[VC] Failed to parse URDF into KDL tree")

        self.chain = tree.getChain(self.base_link, self.tip_link)
        self.nj = self.chain.getNrOfJoints()
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'] # FORCE SETTING JOINT NAMES FOR IRB120

        # Joint limits from URDF (aligned with self.joint_names)
        lower, upper = [], []
        for name in self.joint_names:
            j = robot.joint_map[name]
            lower.append(j.limit.lower)
            upper.append(j.limit.upper)
        self.joint_lower = np.array(lower)
        self.joint_upper = np.array(upper)

        # 2) KDL solvers
        # ---------------------------------------------------------------------
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

        # Damped (weighted) velocity IK if available, else pseudoinverse
        self.ik_solver_vel = kdl.ChainIkSolverVel_wdls(self.chain)
        self.ik_solver_vel.setLambda(1e-2)  # damping factor
        rospy.loginfo("[VC] Using ChainIkSolverVel_wdls (damped IK).")
            
        # ---- NEW: joint-limited position IK (NR_JL) ----
        q_min = kdl.JntArray(self.nj)
        q_max = kdl.JntArray(self.nj)
        for i in range(self.nj):
            q_min[i] = self.joint_lower[i]
            q_max[i] = self.joint_upper[i]

        self.ik_solver_pos = kdl.ChainIkSolverPos_NR_JL(
            self.chain, q_min, q_max, self.fk_solver, self.ik_solver_vel, 100, 1e-4
        )
        rospy.loginfo("[VC] Using ChainIkSolverPos_NR_JL for position IK with joint limits.")


        # 3) ROS I/O
        # ---------------------------------------------------------------------
        self._state_lock = Lock()
        self._q          = None  # np.array, aligned with self.joint_names
        self._last_pose  = None  # PoseStamped

        self.joint_sub = rospy.Subscriber(joint_state_topic, JointState, self._joint_state_cb, queue_size=1)
        self.vel_pub   = rospy.Publisher(vel_cmd_topic, Float64MultiArray, queue_size=1)
        # Unified FK output for logger and other nodes
        self.fk_pub = rospy.Publisher("/com_3d/tip_pose_fk", PoseStamped, queue_size=200)
        # Annoying... Need a retraction phase publisher for the estimator... I would prefer this somewhere else
        self._retract_pub = rospy.Publisher("/com_3d/retract_phase", Bool, queue_size=1, latch=False)

        self.rate = rospy.Rate(self.rate_hz)

        # Wait for first joint state before returning
        rospy.loginfo("[VC] Waiting for JointState...")
        while not rospy.is_shutdown():
            with self._state_lock:
                if self._q is not None: break
            self.rate.sleep()
        rospy.loginfo("[VC] Joint State Found! Ready.")


    # =====================================================================
    #   INTERNAL HELPERS
    # =====================================================================
    def _publish_retract(self, state: Bool):
        self._retract_pub.publish(state)

    def _to_kdl_jnt_array(self, q_np):
        qa = kdl.JntArray(self.nj)
        for i in range(self.nj): qa[i] = float(q_np[i])
        return qa
    
    def _joint_state_cb(self, msg: JointState):
        """ Get joint states, calculate FK, and publish PoseStamped."""
        # Build q aligned with self.joint_names
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        q = np.zeros(self.nj)
        for i, jn in enumerate(self.joint_names):
            if jn in name_to_idx: q[i] = msg.position[name_to_idx[jn]]

        # Compute local FK
        q_kdl = self._to_kdl_jnt_array(q) # convert to KDL
        F = kdl.Frame()
        self.fk_solver.JntToCart(q_kdl, F)

        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        ps.header.frame_id = self.base_link
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = F.p
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = F.M.GetQuaternion()

        with self._state_lock:
            self._q = q
            self._last_pose = ps
        self.fk_pub.publish(ps)

    def _get_tip_pose(self):
        """Return (xyz_np, quat_kdl, age_sec) from last calculated FK pose."""
        with self._state_lock:
            if self._last_pose is None: return None, None, None
            
            ps = self._last_pose
            age = (rospy.Time.now() - ps.header.stamp).to_sec()
            if age > self.ee_max_age:
                rospy.logwarn_throttle(1.0, f"[VC] FK Pose stale: {age*1000:.2f}ms.")
                return None, None, age

            xyz = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
            o = ps.pose.orientation
            return xyz, kdl.Rotation.Quaternion(o.x, o.y, o.z, o.w), age

    def _publish_velocity(self, q_dot_cmd):
        """ Publish joint velocity command to the robot."""
        self.vel_pub.publish(Float64MultiArray(data=q_dot_cmd.tolist()))

    def _ik_velocity(self, q_np, twist_np):
        """Use KDL's velocity IK solver to map twist -> q_dot."""
        q_kdl = self._to_kdl_jnt_array(q_np)

        v = kdl.Vector(twist_np[0], twist_np[1], twist_np[2])
        w = kdl.Vector(twist_np[3], twist_np[4], twist_np[5])
        twist_kdl = kdl.Twist(v, w)
        dq_kdl = kdl.JntArray(self.nj)
        ret = self.ik_solver_vel.CartToJnt(q_kdl, twist_kdl, dq_kdl)
        if ret < 0:
            rospy.logwarn_throttle(1.0, f"[VC] IK vel solver failed: {ret}; zeroing vel")
            return np.zeros(self.nj)

        dq = np.zeros(self.nj)
        for i in range(self.nj):
            dq[i] = dq_kdl[i]
        return dq
    
    def _kdl_rotation_from_quat(self, quat):
        """
        Return PyKDL.Rotation from quaternion [x,y,z,w].
        """
        quat = np.asarray(quat, dtype=float).reshape(-1)
        if quat.shape != (4,):
            raise ValueError("quat must be length 4 [x, y, z, w]")

        n = float(np.linalg.norm(quat))
        if n < 1e-12:
            raise ValueError("quat has ~zero norm")
        quat = quat / n

        return kdl.Rotation.Quaternion(
            float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
        )

    def _apply_q_qdot_limits(self, q_curr, q_dot_cmd):
        """
        Very simple joint position- and velocity-limit avoidance:
        - Predict next position
        - If we are moving further out of the safe region, zero that joint's velocity.
        """
        q_dot_cmd= np.asarray(q_dot_cmd, dtype=float).reshape(self.nj)

        # 1) Clamp to velocity limits
        lim = np.full(self.nj, self.max_joint_vel, dtype=float)
        q_dot_cmd = np.clip(q_dot_cmd, -lim, lim)

        # 2) Predict next position and check limits
        q_next = q_curr + q_dot_cmd * self.dt
        lo_safe = self.joint_lower + self.joint_safety_margin
        hi_safe = self.joint_upper - self.joint_safety_margin

        for i in range(self.nj):
            if q_next[i] < lo_safe[i] and q_dot_cmd[i] < 0.0:
                q_dot_cmd[i] = 0.0
                rospy.logwarn_throttle(1.0, f"[VC] {self.joint_names[i]} LOWER limit; zeroing vel.")
            if q_next[i] > hi_safe[i] and q_dot_cmd[i] > 0.0:
                q_dot_cmd[i] = 0.0
                rospy.logwarn_throttle(1.0, f"[VC] {self.joint_names[i]} UPPER limit; zeroing vel.")
        
        # 3) Final velocity clamp (in case zeroing pushed us over)
        q_dot_cmd = np.clip(q_dot_cmd, -lim, lim)
        return q_dot_cmd

    def _floor_constraint_isok(self, q_np):
        """Return True if fingertip Z is above floor_z_margin. q_np: current or future joint angles."""
        F = kdl.Frame()
        self.fk_solver.JntToCart(self._to_kdl_jnt_array(q_np), F)
        isok = F.p.z() >= self.floor_z_margin
        if not isok:
            rospy.logwarn(f"[VC] Next joint q violates floor constraint: {self.floor_z_margin}. Zeroing velocities.")
        return isok

    # =====================================================================
    #   PUBLIC API
    # =====================================================================

    def move_to_joint_positions(self, q_target, timeout=5.0, tol=4e-3, bypass_floor=False):
        """
        Drive joints to q_target using a simple velocity PID-controller.

        q_target: list/np.array of target joint angles (rad),
                  same order as self.joint_names.
        timeout:  seconds before giving up.
        tol:      norm of joint error to consider 'done'.
        bypass_floor: if True, bypass the floor_z safety check.
        """
        q_target = np.asarray(q_target, dtype=float)

        # ---------- SAFETY: check floor_z constraint ----------
        if not self._floor_constraint_isok(q_target):
            raise RuntimeError(f"[VC] Refusing move_to_joint_positions to target {q_target}")
        # -----------------------------------------------------

        kp_joints = self.kp_joints
        kd_joints = self.kd_joints
        ki_joints = self.ki_joints

        rospy.loginfo(f"[VC] move_to_joint_positions:\n{q_target}")

        _wait_for_egm_active(target_active=True, timeout=5.0)

        start_time = rospy.Time.now()
        stop_reason = None

        # err_prev = np.zeros(6, dtype=float)
        err_I    = np.zeros(6, dtype=float)
        q_prev   = None #np.zeros(self.nj, dtype=float)
        i_zone   = 0.15 # rad
        within_since = None
        dwell_time = 0.2 # seconds to remain within tol before stopping

        ## =========================== MOTION LOOP ============================
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            with self._state_lock:
                q_curr = self._q.copy()
            if q_curr is None:
                self.rate.sleep()
                continue

            if elapsed > timeout:
                rospy.logwarn(f"[VC] move_to_joint_positions timed out. Joint errors:\n"
                              f"{q_target - q_curr}, norm={np.linalg.norm(q_target - q_curr)}")
                break

            # ----- PID control -----
            # Compute error
            err = q_target - q_curr
            # Check for convergence
            if np.linalg.norm(err) < tol:
                if within_since is None:
                    within_since = rospy.Time.now()
                elif (rospy.Time.now() - within_since).to_sec() >= dwell_time:
                    rospy.loginfo("[VC] Joint target reached. Joint errors:\n"
                                  f"{q_target - q_curr}")
                    stop_reason = 1 # 'good' stop
                    break
            else:
                within_since = None


            # Integral anti-windup
            # err_I += err * self.dt
            err_I += np.where(np.abs(err) < i_zone, err * self.dt, 0.0) # Only integrate inside i_zone (close to target)
            err_I = np.clip(err_I, -self.I_max, self.I_max)
            
            # Derivative
            # err_D = (err - err_prev) / self.dt
            
            if q_prev is None:
                q_dot_meas = np.zeros(self.nj, dtype=float) # Skip derivative on first iteration (avoid spike)
            else:
                q_dot_meas = (q_curr - q_prev) / self.dt
            
            q_prev = q_curr.copy()

            # vel_raw = (kp_joints * err) + (ki_joints * err_I) + (kd_joints * err_D)
            vel_raw = (kp_joints * err) - (kd_joints * q_dot_meas) + (ki_joints * err_I)
            # err_prev = err.copy()

            # -----------------------

            # Enforce max joint velocity scaling
            # max_abs = float(np.max(np.abs(vel_raw)))
            # if max_abs > self.max_joint_vel:
            #     scale = self.max_joint_vel / max_abs
            #     vel_raw *= scale
            
            q_dot_cmd = vel_raw
            q_dot_cmd = self._apply_q_qdot_limits(q_curr, q_dot_cmd)
            # -------------- SAFETY: check floor_z constraint ----------
            if not bypass_floor and not self._floor_constraint_isok(q_curr + q_dot_cmd * self.dt):
                stop_reason = "floor_z constraint violated."
                break

            self._publish_velocity(q_dot_cmd)
            self.rate.sleep()

        ## =========================== END MOTION LOOP ============================

        # Stop motion
        self._publish_velocity(np.zeros(self.nj))
        _wait_for_egm_active(target_active=False, timeout=5.0)

        return True if stop_reason == 1 else False


    def cartesian_velocity(self, v, w, duration, force_watcher=None, lock_orient=True, lock_z=True, pub_retract=False):
        """
        Apply a constant Cartesian twist for 'duration' seconds.

        v: [vx, vy, vz] in BASE frame (m/s)
        w: [wx, wy, wz] in BASE frame (rad/s)
        duration: seconds
        """
        v, w = np.array(v), np.array(w)
        xyz_init, M_des, _ = self._get_tip_pose()
        if xyz_init is None or M_des is None: raise RuntimeError("[VC] No FK for Cartesian start")
        z_des = xyz_init[2]

        if not np.allclose(w, 0.0, atol=1e-6):
            lock_orient = False
            rospy.logwarn("[VC] Non-zero angular velocity requested; disabling orientation lock.")

        rospy.loginfo(f"[VC] Cartesian requested: v={v}, w={w}, duration={duration}")

        _wait_for_egm_active(target_active=True, timeout=5.0)

        start_time = rospy.Time.now()
        early_stop_reason = None # For logging

        ## ============================ CARTESIAN MOTION LOOP ============================
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > duration: 
                early_stop_reason = None # Normal completion
                break

            if force_watcher:
                # Allow initial settling, then record baseline. Until then, force it back to MONITOR
                if elapsed < 0.1:
                        force_watcher.reset(force_state="MONITOR")
                else:
                    rospy.loginfo_once("[VC] Force watcher is active and Monitoring!!")
                    if force_watcher.trigger:
                        early_stop_reason = 1 # 'good' stop
                        break

            with self._state_lock:
                q_curr = self._q.copy()
            if q_curr is None:
                self.rate.sleep()
                continue

            # ----------- Cartesian corrections (orient and z-lock) -----------
            xyz_curr, M_curr, _ = self._get_tip_pose()
            if xyz_curr is None or M_curr is None:
                rospy.logwarn_throttle(1.0, "[VC] No FK during Cartesian motion. Stopping motion for one cycle.")
                self._publish_velocity(np.zeros(self.nj))
                self.rate.sleep()
                continue

            v_cmd, w_cmd = v.copy(), w.copy()

            # FIRST build the linear component of the twist command (with z lock if enabled)
            if lock_z:
                vz_corr = self.kp_zlock * (z_des - xyz_curr[2]) # simple P control on Z
                # vz_corr = np.clip(vz_corr, -0.05, 0.05) # OPTIONAL clamp
                v_cmd[2] += vz_corr

            # SECOND determine angular component (w_cmd)
            if lock_orient: # ONLY runs if w_des was zero
                err_rot = (M_curr.Inverse() * M_des).GetRot() # rot vec representation
                w_cmd = self.kp_orient * np.array([err_rot[0], err_rot[1], err_rot[2]], dtype=float)
            # ---------------------------------------------------------------

            # Compute joint velocities via KDL velocity IK
            q_dot_cmd = self._ik_velocity(q_curr, np.concatenate([v_cmd, w_cmd]))
            q_dot_cmd = self._apply_q_qdot_limits(q_curr, q_dot_cmd)

            # -------------- SAFETY: check floor_z constraint ----------
            if not self._floor_constraint_isok(q_curr + q_dot_cmd * self.dt):
                early_stop_reason = "floor_z constraint violated."
                break

            self._publish_velocity(q_dot_cmd)
            if pub_retract:
                self._publish_retract(Bool(data=True))
            self.rate.sleep()
        ## ============================ END CARTESIAN MOTION LOOP ============================

        # Stop after duration
        self._publish_velocity(np.zeros(self.nj))
        if pub_retract:
            self._publish_retract(Bool(data=False))

        # Record how long this took so we can 'undo' it in parent function
        self.last_cartesian_duration = (rospy.Time.now() - start_time).to_sec()

        _wait_for_egm_active(target_active=False, timeout=5.0)
        
        if early_stop_reason is None:
            return True
        if early_stop_reason==1:
            rospy.loginfo(f"[VC] Stopped due to force < f_safe.")
            return True
        else:
            rospy.logwarn(f"[VC] Stopped due to: {early_stop_reason}")
            return False


    def move_to_pose(self, xyz, quat, timeout=5.0, tol=4e-3, q_seed=None):
        """
        Solve IK for (xyz, quat) and then move there using move_to_joint_positions.

        xyz, quat: pose in BASE frame.
        ************************** UNTESTED UNTESTED UNTESTED *******************************
        """
        q_target = self.ik_position(xyz, quat=quat, q_seed=q_seed)

        success = self.move_to_joint_positions(
                q_target=q_target,
                timeout=timeout,
                tol=tol,
                bypass_floor=False
            )
        return success

    def ik_position(self, xyz, quat, q_seed=None):
        """
        Solve joint positions for a desired pose (xyz, quat(xyzw)) using joint-limited IK.

        xyz, quat in BASE frame.
        q_seed: optional initial guess (np.array length nj). If None, use current q.
        Returns: np.array length nj (joint angles) or raises RuntimeError on failure.
        """
        xyz = np.asarray(xyz, dtype=float)
        if xyz.shape != (3,):
            raise ValueError("xyz must be length 3")
        
        R = self._kdl_rotation_from_quat(quat)
        p = kdl.Vector(xyz[0], xyz[1], xyz[2])
        F_des = kdl.Frame(R, p)

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
