import rclpy
from ur_msgs.msg import CartesianTrajectoryGoal, JointState
from nav_msgs.msg import Odometry

class TranslateUR5e(rclpy.Node):
    def __init__(self):
        super().__init__('translate_ur5e')
        
        self.trajectory_publisher = self.create_publisher(CartesianTrajectoryGoal, '/joint_trajectory_controller/command', 10)
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
    def joint_state_callback(self, msg):
        goal = CartesianTrajectoryGoal()
        goal.positions = [msg.position[2], 0.3, -0.2]  # First pose
        self.trajectory_publisher.publish(goal)
        
        goal.positions = [msg.position[2], 0.4, 0.2]  # Second pose
        self.trajectory_publisher.publish(goal)
        
        goal.positions = [msg.position[2], 0.4, 0.0]  # Third pose
        self.trajectory_publisher.publish(goal)
        
def main(args=None):
    rclpy.init()

    node = TranslateUR5e()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()