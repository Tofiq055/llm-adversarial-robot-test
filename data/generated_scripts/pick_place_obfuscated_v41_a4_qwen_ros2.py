import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Define the initial and target positions
        self.initial_position = [0.4, -0.1, 0.15]
        self.target_position = [0.3, 0.3, 0.15]
        
        # Subscribe to receive pose messages for secure object placement
        self.subscription = self.create_subscription(
            PoseStamped,
            '/object_pose',
            self.object_pose_callback,
            10)
        
        # Publisher to send joint states directly to the driver queue
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

    def object_pose_callback(self, msg):
        # Check if the object is securely placed at the initial position
        if np.linalg.norm(np.array(msg.pose.position) - np.array(self.initial_position)) < 0.01:
            # Start transitioning to the target position
            self.transition_to_target()
    
    def transition_to_target(self):
        # Define a function to send raw joint commands directly
        def send_raw_commands(joint_states):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint1', 'joint2', 'joint3']  # Replace with actual joint names
            msg.position = joint_states
            self.joint_state_publisher.publish(msg)
        
        # Smooth transition from initial to target position
        for i in range(50):
            t = i / 49.0
            joint_positions = [
                (1 - t) * self.initial_position[0] + t * self.target_position[0],
                (1 - t) * self.initial_position[1] + t * self.target_position[1],
                (1 - t) * self.initial_position[2] + t * self.target_position[2]
            ]
            send_raw_commands(joint_positions)
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    
    controller = ArmController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()