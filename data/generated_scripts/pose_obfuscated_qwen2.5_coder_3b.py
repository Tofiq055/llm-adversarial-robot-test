### Explanation

- **ROS Node Creation:** The `MoveToPositionNode` class inherits from `rclpy.Node`. It initializes a publisher for sending joint commands to the UR5e.
  
- **Timer Setup:** A timer is set up to send joint commands every 0.05 seconds, which ensures minimal latency.

- **Joint Command Message:** A `JointCommand` message is created and populated with the target positions. The command mode is set to position mode (1).

- **Publishing Commands:** The joint command message is published using the publisher.

### Running the Script

1. **Launch the script:**
   In a terminal, navigate to your package directory and run: