# Import required libraries
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from ur_description.srv import SolveIK
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, JointTrajectory
from moveit_msgs.msg import Constraints, MoveItErrorCodes, PositionConstraints
from std_msgs.msg import Header
import actionlib

# Define the objective pose
objective_pose = PoseStamped()
objective_pose.header = Header(stamp=rospy.Time(), frame_id='base_link')
objective_pose.pose = Pose()
objective_pose.pose.position = Point(0.3, 0.2, 0.4)
objective_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=1.0, w=1.0)

# Define the service client to solve IK
ik_service = rospy.ServiceProxy('execute_cartesian_path', SolveIK)

# Define the target state vector with raw joint commands
target_state = JointTrajectory()
target_state.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
target_state.points = []

# Define the joint position for each point in the trajectory
for _ in range(len(objective_pose.pose.position)):
    point = Point()
    point.x, point.y, point.z = objective_pose.pose.position.x, objective_pose.pose.position.y, objective_pose.pose.position.z
    point.z += 0.05 # Add a small offset to the z position for collision checking
    target_state.points.append(JointTrajectoryPoint())
    target_state.points[-1].positions = [0.2, -0.3, 1.74532925199, 1.685410574773, 2.414213562373] # Example joint angles for the UR5e

# Define a trajectory action client
trajectory_client = actionlib.SimpleActionClient('joint_trajectory_action', JointTrajectoryAction)
trajectory_client.wait_for_server()

# Send the target state vector to the action server
goal = JointTrajectoryGoal(target_state=target_state)
trajectory_client.send_goal(goal)
trajectory_client.wait_for_result()