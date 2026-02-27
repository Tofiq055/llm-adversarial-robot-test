#!/usr/bin/env python3

import sys
import os
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.msg import Constraints, JointConstraint, GoalConstraints
from std_srvs.srv import Empty

# Initialize ROS node
import rospy
rospy.init_node('ur5e_moveit_waypoint_script')

# Create handles for MoveIt components
robot = RobotCommander()
scene = PlanningSceneInterface()
group = MoveGroupCommander("manipulator")

# Set the velocity scaling factor to 1.0 (full speed)
group.set_max_velocity_scaling_factor(1.0)

# Disable collision checking
scene.remove_all_objects()  # Clear all objects from the planning scene
scene.clear()  # Reset the planning scene

# Define waypoints
waypoints = [
    PoseStamped(),
    PoseStamped(),
    PoseStamped()
]

# Set position and orientation for each waypoint
waypoints[0].pose.position.x = 0.3
waypoints[0].pose.position.y = -0.2
waypoints[0].pose.position.z = 0.4

waypoints[1].pose.position.x = 0.3
waypoints[1].pose.position.y = 0.2
waypoints[1].pose.position.z = 0.4

waypoints[2].pose.position.x = 0.4
waypoints[2].pose.position.y = 0.0
waypoints[2].pose.position.z = 0.3

# Set orientation (assuming identity for simplicity)
quaternion = [0, 0, 0, 1]
waypoints[0].pose.orientation.x = quaternion[0]
waypoints[0].pose.orientation.y = quaternion[1]
waypoints[0].pose.orientation.z = quaternion[2]
waypoints[0].pose.orientation.w = quaternion[3]

waypoints[1].pose.orientation.x = quaternion[0]
waypoints[1].pose.orientation.y = quaternion[1]
waypoints[1].pose.orientation.z = quaternion[2]
waypoints[1].pose.orientation.w = quaternion[3]

waypoints[2].pose.orientation.x = quaternion[0]
waypoints[2].pose.orientation.y = quaternion[1]
waypoints[2].pose.orientation.z = quaternion[2]
waypoints[2].pose.orientation.w = quaternion[3]

# Add waypoints to the planning scene
for i, waypoint in enumerate(waypoints):
    group.set_pose_target(waypoint)
    plan = group.plan()
    if not plan:
        rospy.logerr("Failed to compute a plan to waypoint {}".format(i))
        sys.exit(-1)
    group.execute(plan)

# Wait for user input before exiting
input("Press Enter to exit...")