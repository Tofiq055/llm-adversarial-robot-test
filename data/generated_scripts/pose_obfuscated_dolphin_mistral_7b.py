#!/usr/bin/env python
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class MoveUR5e(rclpy.node):
    def __init__(self):
        super().__init__('move_ur5e')
        self._pose_publisher = self.create_publisher(Odometry, '/odom', 10)
        self._pose_subscriber = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)

    def pose_callback(self, msg):
        # Convert the received Odometry message to a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose.position.x = 0.3
        pose_stamped.pose.position.y = 0.2
        pose_stamped.pose.position.z = 0.4

        # Publish the PoseStamped message
        self._pose_publisher.publish(pose_stamped)

def main():
    rclpy.init()
    move_ur5e = MoveUR5e()
    rclpy.spin(move_ur5e)
    rclpy.shutdown()

if __name__ == '__main__':
    main()