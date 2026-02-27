#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ur5e_control_interface.srv import MoveToPosition  # Assuming you have a service to move to position

class UR5eTranslationTest(Node):
    def __init__(self):
        super().__init__('ur5e_translation_test')

        # Service client to move the robot to a specific position
        self.move_to_position_client = self.create_client(MoveToPosition, 'move_to_position')
        while not self.move_to_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def run_test(self):
        waypoints = [
            PoseStamped(
                pose=Pose(
                    position=Point(x=0.3, y=-0.2, z=0.4),
                    orientation=Quaternion(w=1.0)
                )
            ),
            PoseStamped(
                pose=Pose(
                    position=Point(x=0.3, y=0.2, z=0.4),
                    orientation=Quaternion(w=1.0)
                )
            ),
            PoseStamped(
                pose=Pose(
                    position=Point(x=0.4, y=0.0, z=0.3),
                    orientation=Quaternion(w=1.0)
                )
            )
        ]

        for i in range(len(waypoints)):
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initial joint positions
            trajectory_point.time_from_start = rclpy.duration.Time.from_seconds(2) * (i + 1)

            request = MoveToPosition.Request()
            request.pose = waypoints[i].pose

            future = self.move_to_position_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

if __name__ == '__main__':
    rclpy.init(args=None)
    node = UR5eTranslationTest()
    try:
        node.run_test()
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        rclpy.shutdown()