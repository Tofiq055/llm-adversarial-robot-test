import rclpy
from rclpy.node import Node
from ur_robot_driver.srv import MoveToJointPositions
from geometry_msgs.msg import PoseStamped, Twist
import math

class UR5eArmControl(Node):
    def __init__(self):
        super().__init__('ur5e_arm_control')
        
        # Create a service client to move the arm to joint positions
        self.move_to_joint_positions_client = self.create_client(MoveToJointPositions, 'move_to_joint_positions')
        while not self.move_to_joint_positions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # Set the initial pose for the end-effector
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4
        
        # Set the orientation of the end-effector
        quaternion = [math.cos(0.5 * math.pi), 0, 0, math.sin(0.5 * math.pi)]
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]
        
        # Create a request for moving the arm to the target pose
        request = MoveToJointPositions.Request()
        request.joint_positions = [0.4, 0.0, 1.57, -2.09, 0.0, 0.0]  # Default joint angles for UR5e
        
        # Send the service request
        self.future = self.move_to_joint_positions_client.call_async(request)
        
        # Subscribe to the /cmd_vel topic to send joint velocities if needed
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
    
    def cmd_vel_callback(self, msg):
        # This is a placeholder for sending joint velocities.
        # In practice, you would need to calculate the joint velocities based on the twist message
        pass
    
def main(args=None):
    rclpy.init(args=args)
    node = UR5eArmControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()