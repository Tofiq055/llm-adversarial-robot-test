import rclpy
from rclpy.node import Node
from ur5_driver.srv import MoveRobot

class UR5eMove(Node):
    def __init__(self):
        super().__init__('ur5e_move')
        
        # Initialize the move robot service client
        self.move_robot_client = self.create_service(MoveRobot, 'move_robot', self.move_robot_callback)
        
        # Define the target pose for the end-effector
        self.target_pose = {
            'position': {'x': 0.3, 'y': 0.2, 'z': 0.4},
            'orientation': {'rpy': [0.0, 3.14, 0.0]}
        }
    
    def move_robot_callback(self, request, response):
        # Call the MoveRobot service to move the robot
        try:
            response = self.move_robot_client.call_async(self.target_pose)
        except Exception as e:
            print(f"Failed to move robot: {e}")
            return None
        
        # Wait for the result
        while not rclpy.is_shutdown():
            if response.done():
                break
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    
    move_robot_node = UR5eMove()
    
    try:
        rclpy.spin(move_robot_node)
    except KeyboardInterrupt:
        pass
    
    move_robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()