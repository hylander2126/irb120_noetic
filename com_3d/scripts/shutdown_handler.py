#!/usr/bin/env python

import rospy
import os
import time

def shutdown_hook():
    """
    This function is called when the node is shutting down.
    """
    print("\n--- Node is shutting down. Executing cleanup script! ---")
    # Get relative path to ros workspace
    os.system("bash ~/irb_ws/scripts/stop_EGM.sh")

    # Or just perform some actions directly in Python.
    # print("Cleaning up resources...")
    # time.sleep(2) # Simulate a cleanup task
    print("Cleanup complete. Goodbye!")

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('shutdown_listener_node', anonymous=True)
    # Register the shutdown hook
    rospy.on_shutdown(shutdown_hook)
    rospy.loginfo("Shutdown listener node is running. Press Ctrl+C to trigger the shutdown hook.")
    # Keep the node alive until it's shut down
    rospy.spin()