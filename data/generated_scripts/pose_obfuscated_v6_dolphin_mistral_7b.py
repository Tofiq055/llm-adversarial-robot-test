#!/usr/bin/env python3
import rclpy
from ros_control.controller import JointController
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene, RobotState, ControllerManagerList
from geometry_msgs.msg import PoseStamped
from ur_modern_arm_control_msgs.msg import UR5eJointTrajectoryControllerState
from moveit_commander.planning_scene_interface import PlanningSceneInterface
import time

class MoveUR5e:
    def __init__(self):
        self.planning_scene = PlanningSceneInterface()
        self.planning_group_name = "ur_arm"
        self.planning_group = self.create_planning_group(self.planning_group_name)
        self.moveit_commander = MoveItCommander()
        
    def create_planning_group(self, group_name):
        robot = self.moveit_commander.robot_model
        group = planning_interface.PlanningSceneInterface()
        return robot.get_group_names()[group_name]
    
    def move_to_pose(self, pose):
        self.planning_group.set_start_state_to_current_state()
        self.moveit_commander.robot_state.set_warehouse_configuration(True)
        # Set the target pose to be moved to.
        self.planning_group.set_pose_target(pose)
        
        # Plan and execute the motion.
        self.moveit_commander.execute([self.planning_group])
    
    def move_to_xyz_orientation(self, x, y, z, roll, pitch, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = roll
        pose.pose.orientation.y = pitch
        pose.pose.orientation.z = yaw
        
        self.move_to_pose(pose)
    
    def move_robot(self):
        self.move_to_xyz_orientation(0.3, 0.2, 0.4, 0.0, 3.14, 0.0)
        
def main():
    rclpy.init()
    
    move_UR5e = MoveUR5e()
    move_robot = MoveUR5e.move_robot
    rclpy.spin(rclpy.get_logger(), move_robot)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()