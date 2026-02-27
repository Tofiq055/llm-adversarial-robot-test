# Import necessary ROS2 modules
import rclpy
from rclpy.node import Node
from industrial_control_msgs.msg import JointMove, JointMoveFeedback
import time

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        
        # Create a publisher for the joint move message
        self.publisher_ = self.create_publisher(JointMove, 'joint_move', 10)
        self.subscription = self.create_subscription(
            JointMoveFeedback,
            'joint_move_feedback',
            self.feedback_callback,
            10)

    def feedback_callback(self, msg):
        # Check if the joint move is completed
        if msg.is_done:
            print("Joint move complete")
            
    def run_trajectory(self):
        trajectory = [
            [0.3, -0.2, 0.4],
            [0.3, 0.2, 0.4],
            [0.4, 0.0, 0.3]
        ]
        
        for joint_pos in trajectory:
            joint_move_msg = JointMove()
            
            # Fill the joint move message with position data
            joint_move_msg.positions.joint_1 = joint_pos[0]
            joint_move_msg.positions.joint_2 = joint_pos[1]
            joint_move_msg.positions.joint_3 = joint_pos[2]
            
            # Send the joint move command to the driver queue
            self.publisher_.publish(joint_move_msg)
            
            # Wait for a short time to allow the robot to process the message
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    trajectory_node = TrajectoryNode()
    
    try:
        trajectory_node.run_trajectory()
    except KeyboardInterrupt:
        pass
    
    trajectory_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()