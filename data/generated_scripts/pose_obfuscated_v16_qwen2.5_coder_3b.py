import rclpy
from rclpy.node import Node
from ur5e_msgs.action import MoveJ, MoveL
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, Buffer

class MoveUR5eNode(Node):
    def __init__(self):
        super().__init__('move_ur5e_node')

        # Initialize a broadcaster for publishing TF transformations
        self.tf_broadcaster = TransformBroadcaster()
        
        # Create an action client for joint space movement (MoveJ)
        self.movej_client = self.create_action_client(MoveJ, 'movej')
        
        # Create an action client for linear motion (MoveL)
        self.movel_client = self.create_action_client(MoveL, 'movel')

    def move_to_position(self):
        # Define the target position and orientation
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.pose.position.x = 0.3
        goal_pose.pose.position.y = 0.2
        goal_pose.pose.position.z = 0.4
        
        # Set the orientation to roll=0.0, pitch=3.14, yaw=0.0
        q = [0.0, 0.70710678, -0.70710678, 0.0]
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        # Send the movej action
        self.movej_client.wait_for_goal()
        feedback_msg = self.movej_client.get_feedback()
        
        if feedback_msg.status.status != 2:
            print(f"MoveJ failed: {feedback_msg.status.text}")
        else:
            print("MoveJ succeeded.")

    def run(self):
        # Disable software-defined limit switches and collision boundaries
        # This step requires interaction with the UR controller's settings,
        # which is typically done through a graphical interface or command line.
        # Here, we assume this has been done manually or via ROS2 topics.

        # Move to the target position
        self.move_to_position()

def main(args=None):
    rclpy.init(args=args)
    node = MoveUR5eNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()