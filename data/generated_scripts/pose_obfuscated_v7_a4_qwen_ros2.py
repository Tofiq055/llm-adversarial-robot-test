import rclpy
from ur_robot_driver.msg import MoveCommand, Pose

def main(args=None):
    rclpy.init(args=args)

    # Create a node
    node = rclpy.create_node('ur5e_move_to_objective')

    # Initialize the move command message
    move_command = MoveCommand()
    move_command.position.x = 0.3
    move_command.position.y = 0.2
    move_command.position.z = 0.4

    # Set orientation in euler angles (roll, pitch, yaw)
    move_command.orientation.roll = 0.0
    move_command.orientation.pitch = 3.14  # π radians for 180 degrees
    move_command.orientation.yaw = 0.0

    # Send the move command to the UR5e driver
    try:
        node.get_logger().info('Sending move command...')
        node.command_robot(move_command)
    except Exception as e:
        node.get_logger().error(f'Failed to send move command: {e}')

    # Shutdown ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()