import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class URControllerNode(Node):
    def __init__(self):
        super().__init__('ur_controller_node')
        
        # Create a publisher to send joint positions to the controller
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_command', 10)
        
        # Set up initial and target waypoints
        self.initial_waypoint = [2.395758, -1.5708, 2.56079, -1.5708, -2.349019, -0.222193]
        self.target_waypoints = [
            [-0.236581, -1.5708, 1.52624, -1.5708, -2.349019, -0.327849],
            [2.395758, -1.5708, 2.56079, -1.5708, -2.349019, -0.222193]
        ]
        
        # Set up the velocity profile
        self.velocity_profile = [
            [0.8] * 6,
            [0.8] * 6,
            [0.8] * 6
        ]

        # Initialize joint positions to initial waypoint
        self.current_joint_positions = self.initial_waypoint

        # Define a counter for waypoints
        self.waypoint_counter = 0
        
        # Create a timer to trigger waypoint control every 0.1 seconds
        self.timer = self.create_timer(0.1, self.send_joint_command)

    def send_joint_command(self):
        if self.waypoint_counter < len(self.target_waypoints):
            target_positions = self.target_waypoints[self.waypoint_counter]
            
            # Calculate the difference between current and target positions
            delta_positions = [target - current for target, current in zip(target_positions, self.current_joint_positions)]
            
            # Calculate velocities based on velocity profile
            velocities = [delta / (0.1) * vel_profile[self.waypoint_counter][i] for i, delta in enumerate(delta_positions)]
            
            # Create the Float64MultiArray message to send joint commands
            command = Float64MultiArray()
            command.data = self.current_joint_positions + velocities
            
            # Publish the joint command
            self.joint_pub.publish(command)
            
            # Update current joint positions for next iteration
            self.current_joint_positions = target_positions
            
            # Increment waypoint counter
            self.waypoint_counter += 1

        else:
            # Stop if all waypoints are reached
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = URControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()