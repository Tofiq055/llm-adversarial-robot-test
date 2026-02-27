import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, JointState
from geometry_msgs.msg import PoseStamped
from moveit_core.robot_model_loader import RobotModelLoader
from tf2_ros import TransformBroadcaster
import time

class UR5ePickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('ur5e_pick_and_place_node')
        
        # Initialize the MoveIt components
        self.move_group = self.create_move_group('ur5e_arm')
        self.robot_model_loader = RobotModelLoader('robot_description')
        self.planning_scene_interface = self.move_group.get_planning_scene_interface()
        
        # Create a TransformBroadcaster for visualizing frames
        self.tf_broadcaster = TransformBroadcaster()

        # Plan and execute each step
        self.plan_and_execute_step([0.4, -0.1, 0.3])
        self.plan_and_execute_step([0.4, -0.1, 0.15], lower=True)
        self.plan_and_execute_step([0.4, -0.1, 0.35])
        self.plan_and_execute_step([0.3, 0.3, 0.35], lower=True)
        self.plan_and_execute_step([0.3, 0.3, 0.15], open_gripper=True)

    def plan_and_execute_step(self, pose, lower=False, open_gripper=False):
        # Set the end-effector target pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.position.z = pose[2]
        
        # Create a MoveGroupGoal and set the end-effector target
        goal = MoveGroupGoal()
        goal.target_pose = goal_pose
        
        # Set velocity scaling factor
        if self.move_group.set_velocity_scaling_factor(0.1):
            print("Velocity scaling factor set to 0.1")
        
        # Set planning time and maximum attempts
        goal.set_planning_time(60.0)
        goal.max_attempts = 5
        
        # Plan the path and execute it
        self.move_group.plan(goal)
        if not self.move_group.execute(goal):
            print("Failed to plan or execute motion")
            return
        
        # Wait for the gripper to close or open
        time.sleep(1)

if __name__ == '__main__':
    rclpy.init(args=None)
    node = UR5ePickAndPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()