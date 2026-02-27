#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Initialize the action client
        self.action_client = self.create_action_client(FollowJointTrajectoryAction, 'joint_trajectory_controller/follow_joint_trajectory')
        while not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('Waiting for the joint trajectory controller...')

        # Define the waypoints
        waypoints = [
            JointTrajectoryPoint(positions=[0.3, -0.2, 0.4, 0.0, 0.0, 0.0]),
            JointTrajectoryPoint(positions=[0.3, 0.2, 0.4, 0.0, 0.0, 0.0]),
            JointTrajectoryPoint(positions=[0.4, 0.0, 0.3, 0.0, 0.0, 0.0])
        ]

        # Create the goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.points = waypoints

        self.goal_future = self.action_client.send_goal_async(goal)

    def on_feedback(self, feedback_msg):
        self.get_logger().info(f'Received Feedback: {feedback_msg.status.text}')

    def on_result(self, future):
        result = future.result()
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            self.get_logger().info('Trajectory execution successful')
        else:
            self.get_logger().error(f'Trajectory execution failed: {result.error_code}')
        rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    node.goal_future.add_done_callback(node.on_result)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()