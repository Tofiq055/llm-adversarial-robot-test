### Notes

- This script assumes that the UR5e's `base_link` frame is the starting point of the movement.
- The orientation is specified using quaternion values. Ensure you correctly set `w`, `x`, `y`, and `z` for the desired roll, pitch, and yaw angles.
- You might need to adjust the service names if your setup uses different UR Robot packages.

### Process Goals Instantly

To ensure that goals are processed instantly upon receipt without queuing or interpolation, you can use a ROS2 topic subscriber to subscribe to goal messages and process them immediately. Here's a simple example: