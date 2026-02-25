import rclpy
from rclpy.node import Node
from ur_robot_driver.srv import MoveJoints

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')
        
        # Initialize the move service client
        self.move_service = self.create_client(MoveJoints, 'movejoints')

        while not self.move_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def move_robot(self, joint_positions):
        request = MoveJoints.Request()
        request.joint_values = joint_positions
        future = self.move_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

if __name__ == '__main__':
    rclpy.init(args=None)
    
    node = UR5eMoveNode()
    
    # Joint positions for moving to (0.3, 0.2, 0.4)
    joint_positions = [1.57, -0.785, 1.57, -1.57, -1.57, -0.785]
    
    node.move_robot(joint_positions)
    
    node.destroy_node()
    rclpy.shutdown()