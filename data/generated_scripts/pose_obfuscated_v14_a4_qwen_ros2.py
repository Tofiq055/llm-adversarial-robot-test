### Explanation

1. **Node Initialization**: The script initializes a ROS2 node named `ur5e_controller`.
2. **Service Client**: It creates a service client for the `move_group/execute_trajectory` service, which is used to move the robot arm.
3. **Target Pose**: It defines the target pose using Euler angles and converts it to a quaternion.
4. **MoveJoints Request**: A `MoveJointsRequest` object is created, with the desired pose as a goal point.
5. **Move Robot Method**: The `move_robot` method calls the service and logs whether the movement was successful or not.

### Running the Script

1. **Launch the UR robot driver**: Ensure the UR robot driver is running on your system.
2. **Run the script**: Execute the script from a terminal: