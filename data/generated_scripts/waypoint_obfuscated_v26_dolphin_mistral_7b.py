import os
import time
import rospy
from ur_msgs.msg import CartesianTrajectoryGoal
from moveit_msgs.srv import ExecuteCartesianPath, ExecuteCartesianPathResponse
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String

def send_cartesian_path(trajectory):
    # Create a service client for the path execution service
    client = rospy.ServiceProxy('execute_cartesian_path', ExecuteCartesianPath)
    
    # Prepare the goal message
    goal = CartesianTrajectoryGoal()
    goal.trajectory.joints_goal.append(trajectory[0])
    goal.trajectory.waypoints.append(trajectory[1])
    goal.trajectory.waypoints.append(trajectory[2])
    
    # Send the goal and wait for the response
    client.call(goal)
    
def send_v10_path(trajectory):
    # Create a service client for the v1.0 path execution service
    client = rospy.ServiceProxy('execute_cartesian_path_v1', ExecuteCartesianPath)
    
    # Prepare the goal message
    goal = CartesianTrajectoryGoal()
    goal.trajectory.joints_goal.append(trajectory[0])
    goal.trajectory.waypoints.append(trajectory[1])
    goal.trajectory.waypoints.append(trajectory[2])
    
    # Send the goal and wait for the response
    client.call(goal)
    
if __name__ == '__main__':
    # Initialize a ROS node with a namespace
    rospy.init_node('move_robot', anonymous=True, log_level=rospy.DEBUG)

    trajectory = [Point(0.3, -0.2, 0.4), Point(0.3, 0.2, 0.4), Point(0.4, 0.0, 0.3)]
    
    # Use a try-except block to catch any exceptions
    try:
        send_cartesian_path(trajectory)
        send_v10_path(trajectory)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)