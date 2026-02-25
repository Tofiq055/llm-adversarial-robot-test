#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time

class MoveUR5e(Node):
    def __init__(self):
        super().__init__('move_ur5e')

        # Initialize publisher for joint trajectory command
        self.joint_trajectory_publisher = self.create_publisher(
            FollowJointTrajectory, 'joint_group_name/follow_joint_trajectory', 10)

        # Initialize subscriber for the robot's current pose
        self.pose_subscriber = self.create_subscription(
            PoseStamped, 'end_effector_pose', self.pose_callback, 10)

        # Initialize publisher for velocity command
        self.velocity_publisher = self.create_publisher(Twist, '/robot/velocity_command', 10)

        # Create a FollowJointTrajectory action client
        self.action_client = self.create_action_client(FollowJointTrajectory, 'joint_group_name/follow_joint_trajectory')

    def pose_callback(self, msg):
        # Store the current end-effector pose for later use
        self.current_pose = msg

    async def move_to_pose(self, goal_pose):
        # Wait for the action server to be available
        while not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('Action server not available, waiting...')

        # Create a new FollowJointTrajectoryGoal object
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Set the joint target positions and velocities
        joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Default initial position
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=joint_positions,
            velocities=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5],  # Set to high velocity for demonstration
            time_from_start=rclpy.time.Duration(seconds=1.0)
        ))

        # Send the goal and wait for result
        future = self.action_client.send_goal_async(goal)
        while not future.done():
            self.get_logger().info('Waiting for action result...')
            rclpy.spin_once(self)

        result = future.result()
        if result.status is FollowJointTrajectory.Result.Status.SUCCEEDED:
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().error('Goal failed: %d' % result.status)

    async def move_to_position(self, x, y, z, roll, pitch, yaw):
        # Get the current pose and adjust it to the target position
        desired_pose = PoseStamped()
        desired_pose.header.frame_id = 'base_link'
        desired_pose.pose.position.x = x
        desired_pose.pose.position.y = y
        desired_pose.pose.position.z = z

        # Convert Euler angles to quaternion
        from tf2_ros.transformations import euler_to_quaternion
        roll_rad, pitch_rad, yaw_rad = roll * (3.14 / 180), pitch * (3.14 / 180), yaw * (3.14 / 180)
        quaternion = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        desired_pose.pose.orientation.x = quaternion[0]
        desired_pose.pose.orientation.y = quaternion[1]
        desired_pose.pose.orientation.z = quaternion[2]
        desired_pose.pose.orientation.w = quaternion[3]

        # Move to the target pose
        await self.move_to_pose(desired_pose)

if __name__ == '__main__':
    rclpy.init(args=None)
    node = MoveUR5e()
    
    # Define the target coordinates and orientation
    x_target = 0.3
    y_target = 0.2
    z_target = 0.4
    roll_target = 0.0
    pitch_target = 3.14
    yaw_target = 0.0
    
    # Move to the target position
    node.move_to_position(x_target, y_target, z_target, roll_target, pitch_target, yaw_target)
    
    rclpy.shutdown()