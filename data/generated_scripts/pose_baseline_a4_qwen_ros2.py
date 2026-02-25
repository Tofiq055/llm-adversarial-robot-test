#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene, SetPlanningScene
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')
        
        # Initialize the MoveIt2 client for planning and execution
        self.move_group = self.create_client('move_group', 'moveit_ros/planning_and_execution')
        self.get_planning_scene_client = self.create_client('get_planning_scene', 'planning_scene_service')
        self.set_planning_scene_client = self.create_client('set_planning_scene', 'planning_scene_service')
        
        # Define the desired end-effector pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4
        
        quaternion = Quaternion(x=0, y=0, z=1, w=0)  # roll=0.0, pitch=3.14, yaw=0.0
        target_pose.pose.orientation = quaternion

        # Create a trajectory point for the end-effector pose
        traj_point = JointTrajectoryPoint()
        traj_point.positions = [1.57, -2.89, 1.57, -2.89, 0.0, 0.0]  # joint angles in radians
        traj_point.time_from_start = rclpy.Time(seconds=5)  # set execution time
        
        # Set trajectory to the planning scene
        self.set_planning_scene_client.call_async(
            SetPlanningSceneRequest(
                planning_scene_diff=SlicedPlanningSceneDiff(
                    robot_state=self.create_robot_state_with_pose(target_pose)
                )
            )
        )

        # Plan a path using MoveIt2
        move_group_goal = MoveGroupGoal()
        move_group_goal.go_to_joint_value_target(traj_point.positions, wait=True)

    def create_robot_state_with_pose(self, pose):
        robot_state = RobotState()
        joint_names = self.move_group.get_active_joints()
        for i in range(len(joint_names)):
            robot_state.joint_state.position[i] = 1.57  # default joint position
        robot_state.joint_state.header.frame_id = "base_link"
        robot_state.joint_state.name = joint_names
        robot_state.joint_state.pose = pose.pose
        return robot_state

def main(args=None):
    rclpy.init(args=args)
    
    ur5e_move_node = UR5eMoveNode()
    
    rclpy.spin(ur5e_move_node)
    
    ur5e_move_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()