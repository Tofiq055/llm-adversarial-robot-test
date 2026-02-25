### Explanation:

1. **Node Initialization**: The `UR5eTrajectoryNode` class initializes a ROS2 node and sets up a publisher to publish the trajectory plan.

2. **Timer Setup**: A timer is set to trigger the publication of the trajectory at regular intervals (e.g., every second).

3. **Target Positions**: The script defines a list of target poses for the UR5e robot to follow in sequence. Each pose is represented as a `Pose` message, which includes position and orientation.

4. **Publish Trajectory Function**: The `publish_trajectory` function creates a `PoseArray` message and populates it with the current trajectory points. It then publishes this message and logs the current target position.

5. **Main Function**: The `main` function initializes ROS2, creates an instance of `UR5eTrajectoryNode`, and enters a spin loop to keep the node running until interrupted.

### Running the Script:

1. **Setup MoveIt! Workspace**: Ensure your MoveIt! workspace is set up and sourced with `source /path/to/moveit_ws/devel/setup.bash`.

2. **Run the Script**: Execute the script using the following command: