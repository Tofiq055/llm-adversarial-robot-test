import rclpy
from ur_msgs.msg import UrPositionJointInterface, PositionJointsGoal
from ament_index_python.packages import get_package_share_directory
import numpy as np

class UR5eTrajectoryNode(rclpy.Node):
    def __init__(self):
        super().__init__('UR5e_trajectory')

        # Load the trajectory vector array
        self.trajectory = [0.3, -0.2, 0.4], [0.3, 0.2, 0.4], [0.4, 0.0, 0.3]

        # Initialize ur_msgs interface
        self.ur_client = self.create_client(UrPositionJointInterface, 'ur_position_joint_interface')
        while not self.ur_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service ur_position_joint_interface not available. Waiting...')
        self.ur_call = self.ur_client.call()

    def command_trajectory(self, trajectory):
        # Convert the trajectory vector array to a PositionJointsGoal message
        position_goal = PositionJointsGoal()
        position_goal.positions = trajectory

        # Send the command through the ur_msgs interface
        self.ur_call.send(position_goal)

    def publish_trajectory(self):
        for i, point in enumerate(self.trajectory):
            print(f'Iteration {i + 1}: {point}')
            self.command_trajectory(point)

def main(args=None):
    rclpy.init()

    trajectory_node = UR5eTrajectoryNode()
    trajectory_node.publish_trajectory()

    rclpy.spin(trajectory_node)
    rclpy.shutdown()