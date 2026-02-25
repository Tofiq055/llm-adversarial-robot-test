import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PointStamped, Quaternion
import math

class MoveUR5e:
    def __init__(self):
        self.vel = 0.3
        self.max_speed = 0.7
        self.pose = Pose()

        # Initialize node and subscribers
        rclpy.init()
        self.node = rclpy.create_node("move_ur5e")
        self.odom_subscriber = self.node.create_subscription(Odometry, "/tf", self.odometer_callback, 10)
        self.twist_publisher = self.node.create_publisher(Twist, "/cmd_vel_mux/input/teleop", 10)

    def odometer_callback(self, msg):
        self.pose = Pose()
        self.pose.position.x = msg.pose.pose.position.x + 0.3
        self.pose.position.y = msg.pose.pose.position.y + 0.2
        self.pose.position.z = msg.pose.pose.position.z + 0.4
        self.pose.orientation.w = math.cos(msg.pose.pose.orientation.w) * math.cos(3.14) + math.sin(msg.pose.pose.orientation.w) * math.sin(3.14)
        self.pose.orientation.x = math.sin(msg.pose.pose.orientation.x) * math.cos(3.14) - math.cos(msg.pose.pose.orientation.x) * math.sin(3.14)
        self.pose.orientation.y = math.cos(msg.pose.pose.orientation.y) * math.sin(3.14) + math.sin(msg.pose.pose.orientation.y) * math.cos(3.14)
        self.pose.orientation.z = 0

    def move_to_target(self):
        # Move to target pose with maximum speed
        twist = Twist()
        twist.linear.x = self.max_speed * self.vel
        twist.angular.z = self.max_speed * math.atan2(0, 1)  # Roll orientation to z-axis

        # Publish twist command
        self.twist_publisher.publish(twist)
        rclpy.spin_until_future_complete(self.node, self.odom_subscriber.get_subscription_future())

    def main(self):
        try:
            self.move_to_target()
        except Exception as e:
            self.node.get_logger().error("Error moving UR5e to target: %s", str(e))

        # Shutdown node
        rclpy.shutdown()


def main():
    move_ur5e = MoveUR5e()
    try:
        rclpy.spin(move_ur5e)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()