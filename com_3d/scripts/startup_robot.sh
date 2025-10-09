#!/bin/bash
set -e

# This script automates the startup sequence for the ABB IRB 120 robot.

# echo ">>> [Startup Script] Waiting for RWS services to be available..."
# rosservice wait-for-service /rws/start_rapid

echo ">>> [Startup Script] Re-arming RAPID + EGM"
rosservice call /rws/stop_rapid "{}"
rosservice call /rws/pp_to_main "{}"
rosservice call /rws/start_rapid "{}"

echo ">>> [Startup Script] Applying EGM settings"
rosrun com_3d egm_settings_apply.py

echo ">>> [Startup Script] Starting EGM in joint mode"
rosservice call /rws/sm_addin/start_egm_joint "{}"

# echo ">>> [Startup Script] Waiting for the controller manager service..."
# rosservice wait-for-service /egm/controller_manager/switch_controller

echo ">>> [Startup Script] Starting joint_trajectory_controller"
rosservice call /egm/controller_manager/switch_controller "start_controllers: ['joint_trajectory_controller']
stop_controllers: []
strictness: 2
start_asap: false
timeout: 0.0"

echo ">>> [Startup Script] Robot startup sequence complete. Ready to receive trajectories."