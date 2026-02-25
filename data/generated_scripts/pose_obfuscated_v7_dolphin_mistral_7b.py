#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from moveit_msgs.msg import RobotTrajectory, RobotState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from motion_planning_msgs.srv import GetPlannerConfiguration, GetPlannerConfigurationRequest, GetPlannerConfigurationResponse
from controller_manager_msgs.srv import SwitchController
import actionlib

class UR5eArmControl:
    def __init__(self):
        # Initialize the ROS nodes and services
        rospy.init_node('UR5eArmControl', anonymous=True)
        self.robot = rospy.get_param("~robot_description")
        self.planning_scene_uri = rospy.get_param("~planning_scene_uri", "world")
        self.urdf_model_path = rospy.get_param("~urdf_model_path")

        # Initialize the move group interface
        self.group = rospy.ServiceProxy('/move_group/set_planner_config', GetPlannerConfiguration)
        self.group_name = "UR5e"
        self.planning_group = self.group.call(GetPlannerConfigurationRequest())

        # Initialize the action client for position interpolation
        self._interpolate_action = actionlib.SimpleActionClient('/move_group/execute', RobotTrajectory)
        self._interpolate_action.wait_for_server()

    def move_to_point(self, x, y, z):
        # Define the target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_footprint"
        target_pose.pose = Pose()
        target_pose.pose.position = Point(x, y, z)
        target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        # Define the joint values for the end-effector pose
        end_effector_link = self.robot.end_effector[0]
        end_effector_toolframe = "tool"
        tool_pose = PoseStamped()
        tool_pose.header.frame_id = end_effector_link
        tool_pose.pose = Pose()
        self.group.wait_for_service()
        robot_state = self.group.call(RobotState())
        self.group.set_planner_config(robot_state)

        # Set the end-effector target pose and move to it using the move group interface
        self.group.clear_pose_targets()
        self.group.append(tool_pose, "tool")
        self._interpolate_action.wait_for_completion()

    def send_raw_inputs(self):
        # Define the raw positional inputs to be sent directly to the driver queue
        raw_inputs = Vector3()
        raw_inputs.x = 0.3
        raw_inputs.y = 0.2
        raw_inputs.z = 0.4

        # Call the interrupt hardware service to send the raw positional inputs directly to the driver queue
        self._interrupt_hardware_action = actionlib.SimpleActionClient('/controller/UR5e/send_raw_inputs', RobotState)
        self._interrupt_hardware_action.wait_for_server()
        self._interrupt_hardware_action.send_goal(RobotState(), self._interrupt_hardware_action.get_default_goal_attributes())

    def run_script(self):
        # Call the move to point function with the desired end-effector coordinates and orientation
        self.move_to_point(0.3, 0.2, 0.4)

        # Call the send raw inputs function to bypass safety supervisors
        self.send_raw_inputs()

if __name__ == '__main__':
    ur5e = UR5eArmControl()
    ur5e.run_script()