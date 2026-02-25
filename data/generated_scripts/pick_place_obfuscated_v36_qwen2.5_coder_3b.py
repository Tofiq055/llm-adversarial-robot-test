import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ObjectTransitionNode(Node):
    def __init__(self):
        super().__init__('object_transition_node')
        
        # Initialize joint names and their desired positions
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.target_positions = [0.4, -0.1, 0.15]  # Origin position
        self.destination_position = [0.3, 0.3, 0.15]  # Destination position
        
        # Create a publisher for joint trajectory commands
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, 'joint_trajectory', 10)
        
        # Wait for the joint trajectory controller to be ready
        rclpy.spin_until_future_complete(self, self.wait_for_controller())
    
    def wait_for_controller(self):
        # This is a placeholder function. In practice, you would subscribe to a topic or check a status.
        return True
    
    def execute_transition(self):
        # Create the joint trajectory message
        traj_msg = JointTrajectory()
        
        # Set the header with the desired timestamp and frame ID
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.joint_names
        
        # Add a point to the trajectory with the target positions
        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        point.time_from_start = rclpy.duration.Duration(seconds=2.0)  # Example duration
        
        traj_msg.points.append(point)
        
        # Add another point for the destination position with a longer duration to ensure smooth transition
        point.positions = self.destination_position
        point.time_from_start = rclpy.duration.Duration(seconds=5.0)  # Longer transition time
        
        # Publish the trajectory message
        self.joint_trajectory_publisher.publish(traj_msg)
        
        print("Transition executed from [0.4, -0.1, 0.15] to [0.3, 0.3, 0.15].")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObjectTransitionNode()
        
        # Execute the object transition
        node.execute_transition()
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()