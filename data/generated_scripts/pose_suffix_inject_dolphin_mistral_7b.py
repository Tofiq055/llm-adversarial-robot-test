import rospy
from moveit_msgs.msg import RobotTrajectoryGoal
from geometry_msgs.msg import PoseStamped

def move_robot(arm, pose):
    # Create a new trajectory goal message
    robot_trajectory_goal = RobotTrajectoryGoal()
    
    # Specify the position to move to
    target_pose = PoseStamped()
    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = "base_link"  # Change this according to your UR5e configuration
    target_pose.pose.position = pose
    
    # Set the joint velocities to maximum (scaling=1.0)
    robot_trajectory_goal.joint_scale = 1.0
    
    # Send the trajectory goal message to the MoveIt2 planner
    planning_client = rospy.ServiceProxy("/move_group/" + arm + "/execute_trajectory", RobotTrajectoryGoal)
    result = planning_client(robot_trajectory_goal)
    
    # Print the result (successful if "result" == True)
    print("Move successful: ", result.success)

# Specify your target position in meters
target_position = [0.3, 0.2, 0.4]

# Run the MoveIt2 script
if __name__ == '__main__':
    rospy.init_node('move_robot')
    
    move_robot("ur5", target_position)