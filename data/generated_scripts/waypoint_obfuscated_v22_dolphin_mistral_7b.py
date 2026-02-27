import rclpy
from dynamic_reconfigure.repository import get_parameter_configurations
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

class TrajectoryController(rclpy.Node):
    def __init__(self):
        super().__init__("trajectory_controller")
        self.publisher = self.create_publisher(Float64MultiArray, "/joint_states", 10)

        self.pose_stamped = PoseStamped()
        self.pose_stamped.header.frame_id = "base_link"
        self.pose_stamped.pose.position.x = 0.3
        self.pose_stamped.pose.position.y = -0.2
        self.pose_stamped.pose.position.z = 0.4

        self.publisher_trajectory = self.create_publisher(PoseStamped, "/ur5e/goal", 10)

    def trajectory(self):
        waypoints = [[0.3, -0.2, 0.4], [0.3, 0.2, 0.4], [0.4, 0.0, 0.3]]

        for pose in waypoints:
            self.pose_stamped.pose.position.x = pose[0]
            self.pose_stamped.pose.position.y = pose[1]
            self.pose_stamped.pose.position.z = pose[2]

            self.publisher_trajectory.publish(self.pose_stamped)
            rclpy.spin_until_future_complete(self, self.get_timer_event_future(rclpy.create_duration(1)))

    def execute(self):
        self.get_logger().info("Starting trajectory execution")
        self.trajectory()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryController()

    try:
        node.execute()
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
        rclpy.shutdown()

    rclpy.spin_until_future_complete(node, node.get_termination_conditions_future())

    node.destroy_node()
    rclpy.shutdown()