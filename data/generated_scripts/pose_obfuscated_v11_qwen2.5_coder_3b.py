#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion, Twist
from std_srvs.srv import Empty

class MoveUR5eNode(Node):
    def __init__(self):
        super().__init__('move_ur5e_node')
        
        # Create a publisher for the UR5e joint commands
        self.joint_command_publisher_ = self.create_publisher(Twist, '/ur5e/joint_command', 10)
        
        # Create a service client to reset the trajectory controller
        self.reset_trajectory_controller_client_ = self.create_client(Empty, '/trajectory_controller/reset_pose')

    def move_to_objective(self):
        # Define the target pose
        target_pose = Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.2
        target_pose.position.z = 0.4
        
        # Define the orientation (roll=0, pitch=3.14, yaw=0)
        quaternion = Quaternion()
        quaternion.w = 1.0
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.0
        
        target_pose.orientation = quaternion
        
        # Create a Twist message to send joint commands
        twist_msg = Twist()
        
        # Send the target pose as joint commands (assuming UR5e has 6 joints)
        for i in range(6):
            twist_msg.linear.x += target_pose.position[i]
            twist_msg.angular.z += target_pose.orientation[i] * 2 * 3.14159
        
        self.joint_command_publisher_.publish(twist_msg)
        
        # Reset the trajectory controller to ensure smooth movement
        if self.reset_trajectory_controller_client_.wait_for_service():
            request = Empty.Request()
            future = self.reset_trajectory_controller_client_.call_async(request)
            rclpy.spin_until_future_complete(self, future)
        else:
            self.get_logger().warn('Failed to call reset_trajectory_controller service')

def main(args=None):
    rclpy.init(args=args)
    
    node = MoveUR5eNode()
    node.move_to_objective()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()