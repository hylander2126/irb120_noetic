# velocity_controller.py (Updated with P-Controller)

#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from abb_robot_msgs.srv import TriggerWithResultCode
import threading
from numpy.linalg import pinv
from tf.transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_matrix
from com_3d.velocity_controller_kinematics import VelController
import tf

# Topics and Services
VEL_CMD_TOPIC = "/egm/joint_group_velocity_controller/command"
JOINT_STATE_TOPIC = "/egm/joint_states"
EGM_START_SRV = "/rws/sm_addin/start_egm_joint"
EGM_STOP_SRV  = "/rws/sm_addin/stop_egm"
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
EE_FRAME = "tool0"
BASE_FRAME = "base_link"


# Global EGM/Logging wrappers
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



class VelocityCommander:
    def __init__(self):
        # Publisher for direct joint velocity commands
        self.vel_pub = rospy.Publisher(VEL_CMD_TOPIC, Float64MultiArray, queue_size=1)
        self.rate = rospy.Rate(100) # Control loop frequency (Hz)
        self._state_lock = threading.Lock()

        # State variables for feedback
        self.current_joint_positions = None # Latest joint positions
        self.joint_state_sub = rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._joint_state_cb, queue_size=1)
        
        # # MoveIT Kinematics Acces (just for Jacobian)
        # self.robot = moveit_commander.RobotCommander()
        # self.group = moveit_commander.MoveGroupCommander("manipulator")
        # self.kinematic_state = self.group.get_current_state()
        # self.joint_model_group = self.robot.get_joint_model_group("manipulator")

        # Control parameters
        self.P_GAIN = 1.0            # Proportional gain (Adjust this!)
        self.D_GAIN = 0.1           # Derivative gain (not used in this simple P-controller)
        self.I_GAIN = 0.05           # Integral gain (not used in this simple P-controller)
        # Control limits
        self.MAX_VELOCITY = 0.5      # Max commanded joint velocity (rad/s)
        self.TOLERANCE = 0.01        # Homing tolerance (rad)
        
    def _joint_state_cb(self, msg: JointState):
        """Callback to update the latest joint positions."""
        with self._state_lock:
            # Handle case where joint order might be different (safer approach)
            name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
            try:
                self.current_joint_positions = np.array([name_to_pos[jn] for jn in JOINT_NAMES])
            except KeyError:
                rospy.logerr_once(f"Joint names in {JOINT_STATE_TOPIC} do not match expected names.")

    # def _get_jacobian(self, q_curr):
    #     """
    #     Retrieves the Jacobian matrix from the MoveIt core using the current joint state.
    #     Returns: 6x6 numpy array (Linear x,y,z; Angular x,y,z)
    #     """
    #     if q_curr is None:
    #         return None
        
    #     # 1. Update the kinematic state with current joint values
    #     self.kinematic_state.set_joint_group_positions(self.joint_model_group, q_curr)
        
    #     # 2. Get the Jacobian matrix
    #     return self.kinematic_state.get_jacobian(self.joint_model_group)
    
    
    # =========================================================================
    # 1. PID-CONTROLLED JOINT MOVEMENT
    # =========================================================================
    def go_to_joint_target(self, q_goal, timeout=10.0):
        """
        Move the robot to a joint target using a simple P-Controller.
        q_goal: list/array of 6 target joint positions (rad)
        """
        q_goal = np.array(q_goal)
        _start_egm()
        rospy.sleep(0.5)
        
        start_time = rospy.Time.now()
        success = False

        # PID state initialization
        integral_error = np.zeros(6)
        previous_error = np.zeros(6)
        last_time = start_time.to_sec()
        
        rospy.loginfo(f"[VelCmd] Moving to joint target: {q_goal}")

        while (rospy.Time.now() - start_time).to_sec() < timeout and not rospy.is_shutdown():
            with self._state_lock:
                if self.current_joint_positions is None:
                    rospy.logwarn_throttle(1.0, "[VelCmd] Waiting for initial joint state...")
                    self.rate.sleep()
                    continue
                q_curr = self.current_joint_positions.copy()

            currrent_time = rospy.Time.now().to_sec()
            dt = currrent_time - last_time
            last_time = currrent_time

            if dt == 0:
                self.rate.sleep()
                continue

            # 1. Calculate Error
            error = q_goal - q_curr
            max_error = np.max(np.abs(error))
            
            # 2. Check for success
            if max_error < self.TOLERANCE:
                rospy.loginfo(f"[VelCmd] Target reached! Max error: {np.max(np.abs(error)):.4f} rad.")
                success = True
                break
                
            # 3. Calculate PID components
            P = self.P_GAIN * error
            
            # Integral term (anti-windup: only integrate if error is large enough)
            integral_error += error * dt
            integral_error = np.clip(integral_error, -0.5, 0.5) # Simple clamping
            I = self.I_GAIN * integral_error

            # Derivative term (filter of smooth D-term IF NEEDED)
            derivative_error = (error - previous_error) / dt
            D = self.D_GAIN * derivative_error

            # 4. Calculate commanded velocity and saturate (Limit) Velocity
            vel_cmd = P + I + D
            vel_cmd = np.clip(vel_cmd, -self.MAX_VELOCITY, self.MAX_VELOCITY)
            
            # 5. Publish
            self.vel_pub.publish(Float64MultiArray(data=vel_cmd.tolist()))
            previous_error = error
            self.rate.sleep()
            
        # Stop the robot (important!)
        self.vel_pub.publish(Float64MultiArray(data=[0.0]*6))
        rospy.loginfo(f"Final error after move: {np.max(np.abs(q_goal - q_curr)):.4f} rad.")
        
        if not success:
            rospy.logwarn("[VelCmd] Timeout or shutdown reached before achieving goal.")
            
        rospy.sleep(0.5)
        _stop_egm()
        return success

    def go_home_velocity(self, timeout=15.0):
        """
        Implement a robust home move using the P-Controller.
        """
        # Assuming your robot's home position is [0, 0, 0, 0, 0, 0]
        HOME_Q = [0.0] * 6
        return self.go_to_joint_target(HOME_Q, timeout=timeout)
    

    # =========================================================================
    # 2. CUSTOM CARTESIAN VELOCITY CONTROL
    # =========================================================================
    def move_cartesian_velocity(self, xyz_vel, rpy_vel, duration):
        # ... (EGM start, timing setup) ...
        
        # Desired end-effector twist (6x1 vector)
        # Note: Order is [wx, wy, wz, vx, vy, vz] (angular, linear)
        X_dot_des = np.array(rpy_vel + xyz_vel).reshape(6,)

        _start_egm()
        rospy.sleep(0.5)

        start_time = rospy.Time.now()
        rospy.loginfo(f"[CartVel] Received Cart. Vel. (w,v): {X_dot_des}")

        controller = VelController()
        
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            with self._state_lock:
                if self.current_joint_positions is None:
                    rospy.logwarn_throttle(1.0, "[CartVel] Waiting for initial joint state...")
                    self.rate.sleep()
                    continue
                q_curr = self.current_joint_positions.copy()

            # 1. Get Jacobian (J) using custom FK/PoE calculation
            # We use the Jacobian in the BASE (Space) Frame: J_s
            J_s = controller.J_geometric(q_curr)
            # rospy.loginfo_throttle(0.5, f"[CartVel] Current Jacobian J_s:\n{np.round(J_s, 3)}")
            
            if J_s is None:
                self.rate.sleep()
                continue

            # Calculate manipulability index
            w = controller.manipulability(J_s)
            if w < 0.02:
                rospy.logwarn_throttle(0.5, f"Low manipulability detected ({w:.3f}), using damped pseudoinverse.")
                # NOTE: DLS will automatically handle the singularity
                
            # 2. Calculate Pseudoinverse (J_s_pinv)
            # Use pinv from numpy.linalg
            try:
                # J_s_pinv = pinv(J_s)
                J_s_pinv = controller.damped_pinv(J_s, lam=0.1)
            except np.linalg.LinAlgError:
                rospy.logwarn("Singularity detected, skipping control step.")
                self.rate.sleep()
                continue

            # 3. Calculate Joint Velocity Command
            # q_dot = J_s_dagger * X_dot_des (space frame velocity control)
            q_dot_cmd = J_s_pinv @ X_dot_des

            rospy.loginfo_throttle(0.5, f"[VelCmd] Commanded joint velocities: \n{np.round(q_dot_cmd, 3)}")

            q_dot_cmd = np.clip(q_dot_cmd, -self.MAX_VELOCITY, self.MAX_VELOCITY)
            
            # 5. Publish
            self.vel_pub.publish(Float64MultiArray(data=q_dot_cmd.tolist()))
            self.rate.sleep()

        # Stop the robot (important!)
        rospy.loginfo("[CartVel] Duration complete. Sending zero velocity command.")
        self.vel_pub.publish(Float64MultiArray(data=[0.0]*6))
        
        rospy.sleep(0.5)
        _stop_egm()
        
        # Return success as motion was executed for the full duration
        return True


# If you want to use this file directly for testing:
if __name__ == '__main__':
    rospy.init_node('velocity_commander_node')
    commander = VelocityCommander()
    rospy.loginfo("Velocity Commander Node ready. Waiting 3s then moving to home position.")
    rospy.sleep(3.0)
    commander.go_home_velocity()
    rospy.loginfo("Homing complete or timeout reached. Shutting down.")
    rospy.spin()