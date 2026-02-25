#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal
from moveit_msgs.msg import MoveItErrorCodes

# Initialize the ROS node
rospy.init_node('pick_and_place_old_school')

# Create a publisher for joint_trajectory_action
joint_trajectory_publisher = rospy.Publisher('/joint_trajectory_action/goal', JointTrajectory, queue_size=10)

# Define the pick and place waypoints
pick_pose = [0.4, -0.1, 0.15]  # Cartesian coordinates for picking
place_pose = [0.3, 0.3, 0.15]   # Cartesian coordinates for placing

# Joint angles for the UR5e robot in radians
joint_angles_pick = [
    2.89728866,
    -1.42148155,
    1.74532925,
    -0.03224488,
    -1.45666121,
    -0.17612092
]

joint_angles_place = [
    2.89728866,
    -1.42148155,
    1.74532925,
    -0.03224488,
    -1.45666121,
    -0.17612092
]

# Create a JointTrajectory message
joint_trajectory = JointTrajectory()
joint_trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
joint_trajectory.points.append(JointTrajectoryPoint(positions=joint_angles_pick, time_from_start=rospy.Duration(0)))
joint_trajectory.points.append(JointTrajectoryPoint(positions=joint_angles_place, time_from_start=rospy.Duration(1)))

# Send the joint trajectory
trajectory_action_goal = FollowJointTrajectoryGoal()
trajectory_action_goal.trajectory = joint_trajectory
trajectory_action_publisher.publish(trajectory_action_goal)

# Wait for the action to complete
while True:
    rospy.sleep(0.1)
    if trajectory_action_publisher.get_num_connections() > 0:
        break

# Print success message
rospy.loginfo("Pick and place completed successfully.")