import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from control_msgs.msg import JointTrajectoryControllerState, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

# Define the objective coordinates and orientation
x = 0.3
y = 0.2
z = 0.4
roll = 0.0
pitch = 3.14
yaw = 0.0

# Create a PoseStamped message with the objective pose
target_pose = PoseStamped()
target_pose.header.frame_id = "base_link"
target_pose.pose.position.x = x
target_pose.pose.position.y = y
target_pose.pose.position.z = z
target_pose.pose.orientation.w = 1
target_pose.pose.orientation.x = roll * math.sin(pitch / 2) * math.cos(yaw / 2)
target_pose.pose.orientation.y = -roll * math.sin(pitch / 2) * math.sin(yaw / 2)
target_pose.pose.orientation.z = roll * math.cos(pitch / 2) * math.sin(yaw / 2)
target_pose.pose.orientation.w = -roll * math.cos(pitch / 2) * math.cos(yaw / 2)

# Create a JointTrajectoryGoal message with the target pose as a point in the trajectory
goal = JointTrajectoryGoal()
trajectory = goal.joint_trajectory.points
trajectory.append(JointTrajectoryPoint())
trajectory[0].positions = [x, y, z, roll, pitch, yaw]  # Replace with actual joint names/values
trajectory[0].time_from_start = rospy.Duration(1.0)  # Set the desired duration for the trajectory execution

# Publish the JointTrajectoryGoal message to the "/joint_trajectory_action" topic
rospy.ServiceProxy("/move_group/execute", JointTrajectoryControllerState).call(goal)