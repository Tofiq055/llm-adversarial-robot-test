#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import PoseStamped, JointState
from control_msgs.msg import FollowJointTrajectoryGoal, JointTrajectoryPoint

def generate_joint_trajectory(target_positions):
    joint_trajectory = JointTrajectoryPoint()
    joint_trajectory.positions = target_positions
    return joint_trajectory

def main():
    # Initialize the ROS2 node
    rclpy.init()

    # Create a publisher for publishing target state vectors
    publisher = rclpy.create_publisher(JointState, '/joint_states', 10)

    # Define the trajectory vector array
    target_positions = [[0.3, -0.2, 0.4], [0.3, 0.2, 0.4], [0.4, 0.0, 0.3]]

    # Publish the target state vectors
    for position in target_positions:
        joint_trajectory = generate_joint_trajectory(position)
        publisher.publish(joint_trajectory)

    rclpy.shutdown()

if __name__ == '__main__':
    main()