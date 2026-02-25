import rclpy
from rclpy.node import Node
from moveit2_msgs.srv import GetPlanningScene
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

class UR5ePickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('ur5e_pick_and_place')
        
        # Initialize MoveIt2 client for planning scene and move group
        self.move_group = self.create_move_group('manipulator')
        
        # Initialize service clients for getting planning scene and following joint trajectory
        self.get_planning_scene_client = self.create_client(GetPlanningScene, 'planning_scene_service')
        self.follow_joint_trajectory_client = self.create_client(FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')
        
        # Set initial configuration for the robot
        self.move_group.set_named_target('home')
        self.move_group.go(wait=True)
        
    def move_to_pre_grasp(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = -0.1
        target_pose.pose.position.z = 0.3
        target_pose.pose.orientation.w = 1.0
        
        self.move_group.set_pose_target(target_pose)
        success = self.move_group.plan()
        
        if success:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = success[1]
            goal.goal_time_tolerance = rospy.Duration(5.0)
            
            future = self.follow_joint_trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            
            return future.result().status == GoalStatus.SUCCEEDED
        else:
            print("Failed to plan trajectory for pre-grasp")
            return False
    
    def lower_and_grasp(self):
        # Lower the arm to the grasp position and close the gripper
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.z = 0.15
        self.move_group.set_pose_target(target_pose)
        success = self.move_group.plan()
        
        if success:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = success[1]
            goal.goal_time_tolerance = rospy.Duration(5.0)
            
            future = self.follow_joint_trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            
            return future.result().status == GoalStatus.SUCCEEDED
        else:
            print("Failed to plan trajectory for lower and grasp")
            return False
    
    def lift_to_place(self):
        # Lift the arm above the place position
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.z = 0.35
        self.move_group.set_pose_target(target_pose)
        success = self.move_group.plan()
        
        if success:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = success[1]
            goal.goal_time_tolerance = rospy.Duration(5.0)
            
            future = self.follow_joint_trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            
            return future.result().status == GoalStatus.SUCCEEDED
        else:
            print("Failed to plan trajectory for lift to place")
            return False
    
    def lower_and_place(self):
        # Lower the arm to the place position and open the gripper
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.z = 0.15
        self.move_group.set_pose_target(target_pose)
        success = self.move_group.plan()
        
        if success:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = success[1]
            goal.goal_time_tolerance = rospy.Duration(5.0)
            
            future = self.follow_joint_trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            
            return future.result().status == GoalStatus.SUCCEEDED
        else:
            print("Failed to plan trajectory for lower and place")
            return False

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5ePickAndPlaceNode()
    
    if not node.move_to_pre_grasp():
        return
    
    if not node.lower_and_grasp():
        return
    
    if not node.lift_to_place():
        return
    
    if not node.lower_and_place():
        return
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()