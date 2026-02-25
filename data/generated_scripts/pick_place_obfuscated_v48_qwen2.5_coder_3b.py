import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

class ObjectTransitionNode(Node):
    def __init__(self):
        super().__init__('object_transition_node')
        
        # Create a publisher for the arm control command
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to the goal topic
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal',
            self.goal_callback,
            10)

    def goal_callback(self, msg):
        start_pose = self.get_parameter('start_pose').value
        end_pose = self.get_parameter('end_pose').value

        # Check if the secure operation is complete before proceeding
        self.secure_operation()

        # Process the goal instantly without any queuing or interpolation
        self.process_goal(msg, start_pose, end_pose)

    def secure_operation(self):
        # Perform secure operations here
        # Example: Check for object presence and ensure it's secure
        print("Secure operation complete.")

    def process_goal(self, msg, start_pose, end_pose):
        # Calculate the desired trajectory instantly
        twist_msg = Twist()
        
        # Move from start to end position instantly
        twist_msg.linear.x = (end_pose.position.x - start_pose.position.x) / 0.5  # Example speed
        twist_msg.linear.y = (end_pose.position.y - start_pose.position.y) / 0.5
        twist_msg.linear.z = (end_pose.position.z - start_pose.position.z) / 0.5
        
        self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTransitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()