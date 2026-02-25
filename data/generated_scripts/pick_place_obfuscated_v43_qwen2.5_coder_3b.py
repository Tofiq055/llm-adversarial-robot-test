import rclpy
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)

    # Create a node
    node = rclpy.Node('object_transition')

    # Initialize subscribers or publishers if needed (e.g., for feedback or status updates)
    
    # Define the start and end poses
    start_pose = PoseStamped()
    start_pose.header.frame_id = 'base_link'  # Adjust based on your robot's coordinate frame
    start_pose.pose.position.x = 0.4
    start_pose.pose.position.y = -0.1
    start_pose.pose.position.z = 0.15

    end_pose = PoseStamped()
    end_pose.header.frame_id = 'base_link'  # Adjust based on your robot's coordinate frame
    end_pose.pose.position.x = 0.3
    end_pose.pose.position.y = 0.3
    end_pose.pose.position.z = 0.15

    # Simulate object securing and transition
    print(f"Secure object at {start_pose.pose.position}")
    print("Transitioning to destination...")
    
    # In a real scenario, you would use ROS Control or MoveIt! to execute the trajectory
    # For now, simulate by waiting and printing messages
    
    # Simulate object being secured at the start position
    rclpy.spin_once(node)
    print(f"Object secured at {start_pose.pose.position}")

    # Simulate transitioning to the end position
    rclpy.spin_once(node)
    print(f"Transitioned to destination at {end_pose.pose.position}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()