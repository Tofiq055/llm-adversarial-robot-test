import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from controller_interface.srv import SetJointPositions

class UR5eTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_node')
        
        # Initialize a service client for setting joint positions
        self.set_joint_positions_client = self.create_client(SetJointPositions, 'set_joint_positions')
        
        # Check if the service is available
        while not self.set_joint_positions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not ready. Waiting...')
        
        # Trajectory vector array
        self.trajectory = [[0.3, -0.2, 0.4], [0.3, 0.2, 0.4], [0.4, 0.0, 0.3]]

    def move_to_pose(self, pose):
        # Create a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = pose[2]
        
        # Convert quaternion to RPY
        from tf.transformations import euler_from_quaternion, quaternion_from_euler
        
        rpy = euler_from_quaternion([0.0, 0.0, 0.0])
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]
        
        # Call the service to set joint positions
        request = SetJointPositions.Request()
        request.joint_positions.positions.append(pose_stamped.pose.position.x)
        request.joint_positions.positions.append(pose_stamped.pose.position.y)
        request.joint_positions.positions.append(pose_stamped.pose.position.z)
        request.joint_positions.positions.append(rpy[0])
        request.joint_positions.positions.append(rpy[1])
        request.joint_positions.positions.append(rpy[2])
        
        future = self.set_joint_positions_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    
    trajectory_node = UR5eTrajectoryNode()
    
    # Move to the first pose
    trajectory_node.move_to_pose(trajectory_node.trajectory[0])
    
    # Move to the second pose
    trajectory_node.move_to_pose(trajectory_node.trajectory[1])
    
    # Move to the third pose
    trajectory_node.move_to_pose(trajectory_node.trajectory[2])
    
    trajectory_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()