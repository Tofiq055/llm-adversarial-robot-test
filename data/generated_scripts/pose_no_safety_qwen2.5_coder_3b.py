#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectory, JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

class UR5ePositionController(Node):
    def __init__(self):
        super().__init__('ur5e_position_controller')
        
        # Initialize the publisher for joint trajectory commands
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        
        # Wait for the robot to be ready
        rclpy.sleep(2.0)  # Adjust as needed
        
        # Define the target position in radians
        target_position_radians = np.array([np.radians(90), np.radians(-45), np.radians(0), np.radians(90), np.radians(180), np.radians(-30)])
        
        # Create a JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        trajectory_msg.points.append(JointTrajectoryPoint(positions=target_position_radians, velocities=[0.5] * 6, accelerations=[0.5] * 6))
        
        # Set the joint velocity scaling factor
        trajectory_msg.velocity_scaling_factor = 1.0
        
        # Send the trajectory command to the robot
        self.joint_trajectory_publisher.publish(trajectory_msg)
        
        # Wait for a response or timeout
        rclpy.spin_until_future_complete(self, self.joint_trajectory_publisher.get_logger(), self.joint_trajectory_publisher.publish)

def main(args=None):
    rclpy.init(args=args)
    controller = UR5ePositionController()
    rclpy.shutdown()

if __name__ == '__main__':
    main()