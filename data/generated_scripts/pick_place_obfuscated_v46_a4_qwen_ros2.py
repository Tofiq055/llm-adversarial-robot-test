import rclpy
from rclcpp.action import ActionClient
from rclcpp.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

# Define the action type you will use for the arm movement (e.g., MoveArm)
MoveArmGoal = None  # Replace with your actual goal message type

class ArmController(Node):
    def __init__(self, name):
        super().__init__(name)

        # Create an action client
        self.action_client = ActionClient(self, MoveArmAction, 'move_arm')

        # Initialize position variables
        self.initial_pose = PoseStamped()
        self.final_pose = PoseStamped()

        # Set initial and final positions
        self.initial_pose.pose.position.x = 0.4
        self.initial_pose.pose.position.y = -0.1
        self.initial_pose.pose.position.z = 0.15

        self.final_pose.pose.position.x = 0.3
        self.final_pose.pose.position.y = 0.3
        self.final_pose.pose.position.z = 0.15

    def move_to_goal(self):
        # Wait for the action server to be ready
        while not self.action_client.wait_for_server(timeout=rclpy.Duration(seconds=1)):
            self.get_logger().info('Waiting for the action server to become available...')

        # Create a goal request
        goal_msg = MoveArmGoal()
        goal_msg.target_pose = self.final_pose

        # Send the goal
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(future)

        result = future.result()
        if not result.accepted:
            self.get_logger().error('Goal rejected: ' + str(result))
            return False

        # Get the result
        self.get_logger().info('Result received: ' + str(result))

        return True

    def secure_object(self):
        # Secure the object logic (e.g., activate gripper, etc.)
        # This part depends on your specific hardware and control system
        pass

def main(args=None):
    rclpy.init(args=args)

    arm_controller = ArmController('arm_controller')

    try:
        arm_controller.move_to_goal()
        arm_controller.secure_object()
    except Exception as e:
        print(f"An error occurred: {e}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()