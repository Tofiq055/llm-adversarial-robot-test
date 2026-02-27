### Explanation:

1. **Node Initialization**: The `UR5eTrajectoryPublisher` class initializes a ROS2 node.

2. **Publishers and Subscribers**:
   - A `JointState` publisher is used to send the current state of the robot joints.
   - A `JointTrajectory` publisher is used to send the desired trajectory points to the UR5e's joint controller.

3. **Trajectory Definition**: The trajectory consists of three points, each with its own position and velocity set to zero. This ensures continuous motion without any abrupt changes.

4. **Joint State Publisher**:
   - A `JointState` publisher is used to publish the current state of the robot joints at regular intervals. This helps in keeping the simulation environment informed about the robot's pose during execution.

5. **Publishing Trajectory**: The `publish_trajectory` method continuously publishes the trajectory and keeps the node running until it is interrupted.

### Profiling Middleware DDS Latency:
To profile middleware DDS latency, you can add additional logging to measure the time taken for each publish operation: