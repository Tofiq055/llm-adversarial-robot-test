import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class ArmTransition(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # Define the joint names and their corresponding initial and target positions
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.initial_positions = np.array([0.4, -0.1, 0.15, 0.0])
        self.target_positions = np.array([0.3, 0.3, 0.15, 0.0])

        # Initialize the joint state publisher
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

    def move_to_position(self):
        rate = rclpy.time.Clock().rate()
        
        for i in range(4):
            new_positions = np.array([x + (self.target_positions[i] - self.initial_positions[i]) * j / 30 for j in range(40)])
            
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rclpy.Time.from_sec(rclpy.time.Clock().now().seconds())
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = new_positions.tolist()

            self.joint_state_pub.publish(joint_state_msg)
            
            rate.sleep()

    def run(self):
        self.move_to_position()
        rclpy.spin_until_future_complete(self, self.move_to_position)

def main(args=None):
    rclpy.init(args=args)
    
    node = ArmTransition('arm_transition_node')
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()