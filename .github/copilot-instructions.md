# AI Coding Agent Instructions for com_3d Package

## Project Overview
**com_3d** is a ROS-based robotics package for real-time 3D center-of-mass (CoM) estimation on an ABB IRB120 robot using push interactions. The system estimates object properties (mass, CoM location) via force/torque sensing and AprilTag vision during controlled pushing motions.

### Architecture Layers
1. **Low-Level Control**: `VelocityController` (cartesian + joint velocity control via EGM hardware interface)
2. **Sensing**: `ForceWatcher` (state machine for contact/release detection) + `fk_tip_publisher` (forward kinematics)
3. **Estimation**: `BatchEstimator` (streaming_estimator.py) - accumulates push/retract data, filters, and fits CoM model
4. **Execution**: `push.py` - orchestrates the full push experiment (prep, execute, retract, log)

## Data Flow & ROS Topic Structure
```
/netft_data_transformed (WrenchStamped)
  ↓ [ForceWatcher processes]
/com_3d/fw_contact_status (Bool)
/com_3d/fw_trigger_status (Bool)

/egm/joint_states (JointState) + /com_3d/tip_pose_global (PoseStamped from fk_tip_publisher)
  ↓ [BatchEstimator accumulates]
/com_3d/online_estimate (Float64MultiArray) [m, zc, quality metrics]

Control Flow:
/com_3d/log_start/stop → sync_logger.py records CSV
/com_3d/est_start/stop → BatchEstimator processes phase
/com_3d/retract_phase → signals retract motion for masking
```

## Critical Developer Patterns

### 1. ROS Node Lifecycle & Parameters
- All nodes use `rospy.get_param()` with defaults (see `FKTipPublisher.__init__` for pattern)
- Parameters are loaded from YAML in `config/` (e.g., `controllers_vel.yaml`, `hardware_egm_vel.yaml`)
- Key parameters: `fs_hz` (500 Hz), `o_obj` (object frame origin), `rc0_known` (contact point offset), `e_hat` (rotation axis)

### 2. Threading & Locks
- Use `threading.Lock()` for thread-safe state updates (see `BatchEstimator` for example)
- ROS Subscriber callbacks don't hold locks during slow operations; snapshot data then release
- Example: `streaming_estimator.py` snapshots arrays in critical section, then filters outside lock

### 3. Batch Estimation Strategy
- **Push Phase**: contact=True, retract=False → accumulate force, angle, EE position
- **Retract Phase**: contact=True, retract=True → separate data accumulation
- **Filtering**: Apply `scipy.signal.filtfilt()` (zero-phase 4th-order butter filter at 5 Hz) to force & angle
- **Masking**: Discard data below θ_min (2°) to skip no-load transients
- **Model Fitting**: Separate `_estimate_params()` calls for push vs retract phases → `tau_model()` and `tau_app_model()`

### 4. Quaternion Handling
- Convention: **xyzw** (scipy Rotation compatible, not wxyz)
- Use `enforce_quat_continuity()` (helper_fns.py) for CSV data with missing tags
- Enforce sign continuity to avoid sudden quaternion flips (check dot product > 0)
- Normalize after operations: `quat_normalize(q) = q / ||q||`

### 5. Coordinate Frames
- **table**: Fixed base/world frame (robot base origin)
- **finger_tip**: End-effector frame (defined in URDF, 35mm z-offset from mounting bracket)
- **object**: CoM frame with e_hat (rotation axis, typically [0, 1, 0]) and rc0_known (contact offset)
- Transform pattern: Use KDL (kdl_parser_py) for URDF parsing, PyKDL for FK/Jacobian computation

### 6. Launch & Hardware Integration
- EGM (Externally Guided Motion) must be active before commanding velocity controllers
- Pattern: `_wait_for_egm_active()` checks `/egm/egm_states`, calls `/rws/sm_addin/start_egm_joint` if needed
- Hardware configs in `config/vel_hardware_base.yaml` (EGM=true) and `config/hardware_egm_vel.yaml`
- Joint state topic default: `/egm/joint_states`; velocity command topic: `/egm/joint_group_velocity_controller/command`

### 7. Velocity Control Loop
- `VelocityController` runs at 100 Hz internally (dt=0.01s)
- Methods: `move_to_joint_positions(q_target, timeout=5.0)` and `cartesian_velocity(v, w, duration)`
- Enforces joint limits (safety margin ±0.05 rad) and floor z-height (≥0.06 m above floor)
- Jacobian clamping: if determinant < 1e-3, velocity command is zeroed (singularity protection)

## Key Files & Their Roles

| File | Purpose |
|------|---------|
| [scripts/push.py](scripts/push.py) | Main experiment orchestrator; defines OBJECT_MOTIONS dict with per-object CoM targets |
| [scripts/streaming_estimator.py](scripts/streaming_estimator.py) | BatchEstimator class; accumulates & fits CoM from push/retract phases |
| [src/com_3d/vel_controller.py](src/com_3d/vel_controller.py) | Low-level cartesian + joint velocity control, KDL-based FK/Jacobian |
| [src/com_3d/force_watcher.py](src/com_3d/force_watcher.py) | Contact/release state machine; publishes Bool triggers on `/com_3d/fw_*_status` |
| [src/com_3d/com_estimation.py](src/com_3d/com_estimation.py) | Model functions: `tau_app_model()` (applied force torque), `tau_model()` (gravity torque) |
| [scripts/fk_tip_publisher.py](scripts/fk_tip_publisher.py) | Subscribes `/egm/joint_states`, publishes tip pose via KDL FK to `/com_3d/tip_pose_global` |
| [src/com_3d/helper_fns.py](src/com_3d/helper_fns.py) | 2100+ lines; quaternion, rotation, robotics utilities (enforce_quat_continuity, axisangle2rot, etc.) |
| [config/*.yaml](config/) | ROS parameters: controllers, hardware, kinematics, motion limits |

## Workflow Commands

### Building & Running
```bash
# Build package (catkin_make or catkin build com_3d)
cd ~/irb_ws && catkin build com_3d

# Launch velocity-controlled bringup (starts EGM, ros_control, state publishers)
roslaunch com_3d bringup_irb120_vel.launch robot_ip:=<ROBOT_IP>

# Run push experiment (requires bringup + object config in push.py)
rosrun com_3d push.py
```

### Logging & Post-Processing
- CSV logs saved to `experiments/` with timestamp_object_height naming
- [post_processing/](post_processing/) contains analysis scripts for fitting summary .txt files
- DEPRECATED/ has legacy scripts; favor current implementations in scripts/

## Testing & Validation Patterns
- **Simulation**: Use Gazebo-based robot model (configured in separate launch files)
- **Logging validation**: Check CSV columns (time, fx, fy, fz, x_ee, y_ee, z_ee, qx, qy, qz, qw, contact, retract)
- **Batch fit validation**: Ensure minimum 15 samples per phase after masking; check residual quality scores

## Common Pitfalls to Avoid
1. **Quaternion sign flips**: Always enforce continuity before rotation operations
2. **Singular configurations**: Check Jacobian determinant before inverting; use clamped velocities
3. **Filter phase lag**: Use `filtfilt()` (zero-phase) not `lfilter()` for post-processing
4. **Timing mismatches**: Ensure ros::Time headers are consistent across message types
5. **Lock contention**: Snapshot data quickly in callbacks; do heavy computation outside critical sections
6. **EGM not active**: Velocity commands silently fail if EGM is off; always check state first

## Conventions & Style
- Parameter names use snake_case (e.g., `o_obj`, `rc0_known`, `e_hat`)
- Topic names under `/com_3d/` namespace (avoid `/` prefix in rosparam lookups)
- Internal state variables use `_` prefix (e.g., `self._lock`, `self._sub`)
- Logging: `rospy.loginfo()` for major events, `rospy.logwarn_throttle()` for repeated warnings
