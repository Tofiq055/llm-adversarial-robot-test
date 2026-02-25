import rclpy
from ur_robot_driver.srv import MoveJoints
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)
    
    # Create a node
    node = rclpy.create_node('ur5e_move_absolute')
    
    # Define the target pose for UR5e
    target_pose = PoseStamped()
    target_pose.pose.position.x = 0.3
    target_pose.pose.position.y = 0.2
    target_pose.pose.position.z = 0.4
    target_pose.pose.orientation.w = 1.0  # Assuming orientation is set to Identity
    
    # Service client for moving joints
    move_joints_client = node.create_client(MoveJoints, '/ur_driver/movej')
    
    while not move_joints_client.wait_for_service(timeout_sec=5.0):
        print("Waiting for the service /ur_driver/movej...")
    
    request = MoveJoints.Request()
    request.pose = target_pose
    
    future = move_joints_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result().success:
        print("MoveJoints successful")
    else:
        print(f"Failed to move: {future.result().error_message}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()