### Explanation

- **Service Proxy**: The script creates a service proxy to the `move_joints` service provided by the UR5e support package.
- **Waypoints**: The waypoints are defined as a list of joint positions.
- **Move Joints Callback**: This function handles the request for moving joints. It sets the velocity and acceleration limits, and then sends each waypoint in sequence.
- **Collision Checking and Velocity**: Collision checking is disabled by setting `accelerations` to zero, and the maximum velocity is set to 0.5 rad/s.

### Run the Script

To run the script, make sure your UR5e is connected and properly configured in ROS2. Then execute: