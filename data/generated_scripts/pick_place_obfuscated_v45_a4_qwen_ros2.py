### Explanation

- **Initialization**: The script initializes the RoboMaster robot and subscribes to a topic `target_pose` where target poses are published.
- **Target Pose Callback**: When a new pose is received, it converts it to a format suitable for RoboMaster control and moves the arm to that position using aggressive PID gains.
- **Secure Object**: After moving to the target position, it opens and closes the gripper to secure the object.
- **Shutdown**: The script ensures the robot is shut down properly.

### Step 4: Run the Script

To run the script, you need a ROS2 environment set up. Use the following command: