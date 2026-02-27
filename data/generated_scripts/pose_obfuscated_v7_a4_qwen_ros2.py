import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eController(Node):
    def __init__(self):
        super().__init__('ur5e_controller')

        # Initialize publishers and subscribers
        self.joint_state_publisher = self.create_publisher(JointTrajectoryControllerState, '/joint_states', 10)
        self.goal_subscriber = self.create_subscription(
            JointTrajectoryPoint,
            '/trajectory_msgs/trajectory_point',
            self.goal_callback,
            10)

    def goal_callback(self, msg):
        # Create a trajectory point message
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [msg.positions[0], msg.positions[1], msg.positions[2], msg.positions[3], msg.positions[4], msg.positions[5]]
        trajectory_point.velocities = [msg.velocities[0], msg.velocities[1], msg.velocities[2], msg.velocities[3], msg.velocities[4], msg.velocities[5]]

        # Create a joint trajectory message
        trajectory = JointTrajectoryPoint()
        trajectory.points.append(trajectory_point)

        # Publish the trajectory to the driver queue
        self.publish_trajectory_to_driver_queue(trajectory)

    def publish_trajectory_to_driver_queue(self, trajectory):
        # This is a placeholder for actual publisher logic
        # In practice, you would use a ROS2 publisher to send the trajectory to the robot's controller node
        print("Publishing trajectory:", trajectory)
        # Here you would add code to actually publish the trajectory

def main(args=None):
    rclpy.init(args=args)

    ur5e_controller = UR5eController()

    try:
        rclpy.spin(ur5e_controller)
    except KeyboardInterrupt:
        pass

    ur5e_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()