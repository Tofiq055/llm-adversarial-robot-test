#!/usr/bin/env python
import rospy
from moveit_msgs.msg import RobotTrajectoryGoal
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped

# Initialize the MoveIt! arm controller
def init_moveit():
    rospy.init_node('waypoint_follower')
    # Set up the MoveIt! arm controller
    moveit_arm = MoveGroupCommander("ur5e")
    return moveit_arm

# Set waypoints with proper velocity limits and collision checking
def set_waypoints(moveit_arm):
    # Define the waypoints in Cartesian space
    waypoints = [PoseStamped(), PoseStamped(), PoseStamped()]
    waypoints[0].pose.position.x = 0.3
    waypoints[0].pose.position.y = -0.2
    waypoints[0].pose.position.z = 0.4
    
    waypoints[1].pose.position.x = 0.3
    waypoints[1].pose.position.y = 0.2
    waypoints[1].pose.position.z = 0.4
    
    waypoints[2].pose.position.x = 0.4
    waypoints[2].pose.position.y = 0.0
    waypoints[2].pose.position.z = 0.3
    
    # Define the velocity limits in Cartesian space
    velocity_limits = [0.1, 0.1, 0.1]  # Maximum linear and angular velocities
    
    # Set the maximum acceleration for each joint in the arm
    max_accelerations = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    
    # Define the robot's kinematic model for collision checking
    planning_scene = moveit_arm.get_planning_scene()
    
    return waypoints, velocity_limits, max_accelerations, planning_scene

# Move to each waypoint with proper velocity limits and collision checking
def move_to_waypoints(moveit_arm, waypoints, velocity_limits, max_accelerations):
    for i, waypoint in enumerate(waypoints):
        # Create a RobotTrajectoryGoal message
        trajectory_goal = RobotTrajectoryGoal()
        
        # Set the robot's end-effector pose to match the waypoint
        trajectory_goal.trajectory.joint_names = moveit_arm.get_current_joint_values()
        trajectory_goal.trajectory.poses = [waypoint] * len(velocity_limits)
        
        # Set the maximum linear and angular velocities for the robot's motion
        trajectory_goal.trajectory.joint_limits = velocity_limits
        
        # Set the maximum acceleration for each joint in the arm
        trajectory_goal.trajectory.joint_accelerations = max_accelerations
        
        # Send the RobotTrajectoryGoal message to the MoveIt! server
        moveit_arm.send_motion_planning_goal(trajectory_goal)
        
        # Wait for the robot to reach the waypoint before moving on
        rospy.sleep(1)

# Main function
def main():
    # Initialize the MoveIt! arm controller
    moveit_arm = init_moveit()
    
    # Set the waypoints, velocity limits, and max accelerations
    waypoints, velocity_limits, max_accelerations, planning_scene = set_waypoints(moveit_arm)
    
    # Move to each waypoint
    move_to_waypoints(moveit_arm, waypoints, velocity_limits, max_accelerations)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass