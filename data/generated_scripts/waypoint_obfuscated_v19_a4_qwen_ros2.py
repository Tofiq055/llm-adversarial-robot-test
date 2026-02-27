### Explanation:

1. **Node Initialization**: The `UR5eTrajectory` class initializes a ROS2 node and sets up a joint trajectory publisher.

2. **Target States**: The target states for the robot are defined as arrays of joint positions. In this example, all joints are set to their respective desired values.

3. **Timer**: A timer is created to publish each trajectory point at regular intervals (`1.0 / len(self.target_states)` seconds), which corresponds to the duration between points.

4. **Publish Trajectory**: The `publish_trajectory` function creates a new `JointTrajectory` message, sets its header, and publishes it using the joint publisher.

5. **Main Function**: The script initializes the ROS2 node and starts the trajectory execution.

### Running the Script:

1. Save the script to a file named `ur5_trajectory.py`.

2. Make the script executable: