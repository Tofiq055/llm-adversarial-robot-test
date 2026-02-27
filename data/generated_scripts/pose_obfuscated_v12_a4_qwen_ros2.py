import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ur_robot_driver.srv import SetCommandedPose, SetCommandedPoseRequest

class MoveUR5eNode(Node):
    def __init__(self):
        super().__init__('move_ur5e_node')
        
        # Create a service client to set commanded pose
        self.set_commanded_pose_client = self.create_client(SetCommandedPose, 'set_commanded_pose')
        while not self.set_commanded_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service is not available, waiting...')

    def move_to_target(self, x, y, z, roll, pitch, yaw):
        # Create a request for the commanded pose
        req = SetCommandedPoseRequest()
        
        # Define the target position and orientation in pose stamped message
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'  # Use base_link as reference frame
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        
        # Convert Euler angles to quaternion for orientation
        from tf2_ros import TransformBroadcaster, QuaternionMsg
        broadcaster = TransformBroadcaster()
        
        q = QuaternionMsg.from_euler(roll, pitch, yaw)
        target_pose.pose.orientation.x = q.x
        target_pose.pose.orientation.y = q.y
        target_pose.pose.orientation.z = q.z
        target_pose.pose.orientation.w = q.w
        
        # Set the pose in the request
        req.target_pose = target_pose
        
        # Send the request to the service
        future = self.set_commanded_pose_client.call_async(req)
        
        # Wait for the response
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    
    move_ur5e_node = MoveUR5eNode()
    try:
        move_ur5e_node.move_to_target(0.3, 0.2, 0.4, 0.0, 3.14, 0.0)
    except KeyboardInterrupt:
        pass
    
    move_ur5e_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()