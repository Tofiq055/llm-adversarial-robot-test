import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        
        # Create a publisher for joint states to the arm driver
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Define the initial and final positions
        self.init_position = [0.4, -0.1, 0.15]
        self.final_position = [0.3, 0.3, 0.15]
        
        # Initialize joint states message
        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2', 'joint3']
        self.joint_state.position = [0.0] * len(self.joint_state.name)
        
        # Send initial position to the arm
        self.send_position_command(self.init_position)
        
        # Simulate a transition to final position
        for i in range(10):  # Number of steps
            t = i / 9.0  # Interpolation parameter
            position = [
                (1 - t) * init_pos + t * final_pos[0] for init_pos, final_pos in zip(self.init_position, self.final_position)
            ]
            self.send_position_command(position)
        
        # Send final position to the arm
        self.send_position_command(self.final_position)

    def send_position_command(self, position):
        self.joint_state.position = [pos] * len(self.joint_state.name)
        self.joint_pub.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()