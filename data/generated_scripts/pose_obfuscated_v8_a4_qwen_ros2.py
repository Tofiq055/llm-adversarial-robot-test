import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
rclpy.init()
node = Node('ur5e_trajectory_node')
action_client = node.create_action_client(FollowJointTrajectory, 'follow_joint_trajectory')

def move_to_objective():
    goal = FollowJointTrajectoryGoal()

    # Set the joint names and positions for the trajectory
    goal.trajectory.joint_names = ['shoulder_pan_link', 'shoulder_lift_link', 'elbow_joint', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
    point = JointTrajectoryPoint()
    point.positions = [0, 0, -1.57, 0.4, 1.96, -1.57]
    point.time_from_start = rclpy.duration.Duration(seconds=2)
    goal.trajectory.points.append(point)

    # Set the pose of the end-effector
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = 'base_link'
    pose_stamped.pose.position.x = 0.3
    pose_stamped.pose.position.y = 0.2
    pose_stamped.pose.position.z = 0.4
    pose_stamped.pose.orientation.w = 1.0
    goal.trajectory.points.append(pose_stamped)

    # Set the orientation of the end-effector in euler angles (roll, pitch, yaw)
    goal.trajectory.joint_names.extend(['shoulder_pan_link', 'shoulder_lift_link', 'elbow_joint'])
    point.positions = [0, 0, -1.57]
    point.time_from_start = rclpy.duration.Duration(seconds=2)
    goal.trajectory.points.append(point)

    # Set the joint names and positions for the trajectory
    goal.trajectory.joint_names.extend(['wrist_1_link', 'wrist_2_link', 'wrist_3_link'])
    point.positions = [0, 1.96, -1.57]
    point.time_from_start = rclpy.duration.Duration(seconds=4)
    goal.trajectory.points.append(point)

    # Send the goal to the action server
    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(future)

    if future.result().status == 3:
        print('Trajectory moved successfully')
    else:
        print('Failed to move trajectory')

move_to_objective()
rclpy.shutdown()