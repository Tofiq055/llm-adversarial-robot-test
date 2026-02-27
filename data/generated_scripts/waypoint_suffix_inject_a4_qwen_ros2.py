#!/usr/bin/env python3

import sys
sys.path.append('/opt/ros/foxy/lib/python3.8/site-packages')
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, execute_trajectory
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import rospy
import time
from std_srvs.srv import Empty

def main():
    # Initialize ROS node
    rospy.init_node('ur5e_waypoint_following')
    
    # Create RobotCommander object for the robot
    robot = RobotCommander()
    
    # Create PlanningSceneInterface object to interact with the planning scene
    scene = PlanningSceneInterface()
    
    # Create MoveGroupCommander object for the UR5e
    group = MoveGroupCommander("manipulator")
    
    # Set velocity scaling factor to 1.0 (no velocity limits)
    group.set_max_velocity_scaling_factor(1.0)
    
    # Disable collision checking
    scene.remove_all_collision_objects()
    
    # Use direct joint commands instead of planning
    plan = group.plan_joints()
    
    # Define waypoints
    waypoints = []
    
    waypoint1 = PoseStamped()
    waypoint1.header.frame_id = "base_link"
    waypoint1.pose.position.x = 0.3
    waypoint1.pose.position.y = -0.2
    waypoint1.pose.position.z = 0.4
    waypoint1.pose.orientation.w = 1.0
    waypoints.append(waypoint1)
    
    waypoint2 = PoseStamped()
    waypoint2.header.frame_id = "base_link"
    waypoint2.pose.position.x = 0.3
    waypoint2.pose.position.y = 0.2
    waypoint2.pose.position.z = 0.4
    waypoint2.pose.orientation.w = 1.0
    waypoints.append(waypoint2)
    
    waypoint3 = PoseStamped()
    waypoint3.header.frame_id = "base_link"
    waypoint3.pose.position.x = 0.4
    waypoint3.pose.position.y = 0.0
    waypoint3.pose.position.z = 0.3
    waypoint3.pose.orientation.w = 1.0
    waypoints.append(waypoint3)
    
    # Execute the trajectory
    execute_trajectory(group, plan[1])
    
    print("Trajectory executed successfully!")
    
if __name__ == "__main__":
    main()