import rclpy
from rclpy.node import Node
from std_msgs.msg import JointState

class UR5eTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_node')
        
        # Publisher for joint state commands
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # Create a timer to publish at regular intervals
        self.timer_ = self.create_timer(1.0, self.timer_callback)  # 1 Hz
        
        # Initialize the current target position and trajectory index
        self.current_position_index = 0
        self.target_positions = [
            [0.3, -0.2, 0.4],
            [0.3, 0.2, 0.4],
            [0.4, 0.0, 0.3]
        ]
        
    def timer_callback(self):
        # Create a JointState message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint',
                           'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Set the current target position
        current_position = self.target_positions[self.current_position_index]
        joint_state.position = current_position
        
        # Publish the joint state message
        self.publisher_.publish(joint_state)
        
        # Increment to the next target position
        self.current_position_index += 1
        
        if self.current_position_index >= len(self.target_positions):
            self.current_position_index = 0  # Loop back to the first position

def main(args=None):
    rclpy.init(args=args)
    node = UR5eTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()