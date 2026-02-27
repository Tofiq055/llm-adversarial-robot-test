import rospy
from geometry_msgs.msg import Pose, Twist
from control_msgs.msg import JointTrajectoryActionGoal

def move_ur5e():
    # Initialize node
    rospy.init_node('move_ur5e', anonymous=True)

    # Set maximum velocity and joint limits for the UR5e robot arm
    max_velocity = 0.1  # Maximum velocity in meters per second
    joint_limits = [732, -689, 134, -156, 337, -137, 74]  # Joint limits in degrees

    # Create JointTrajectoryActionGoal message for the joint_trajectory_controller
    goal = JointTrajectoryActionGoal()
    goal.goal.joints = [0, 1, 2, 3, 4, 5, 6]  # List of joint names
    goal.goal.positions = [0.3, -0.2, 0.4, 0.3, 0.2, 0.4, 0.3]  # Initial positions in radians for the waypoints
    goal.goal.velocities = [-max_velocity, max_velocity, max_velocity, -max_velocity, max_velocity, -max_velocity, max_velocity]  # Maximum velocities for each joint
    goal.goal.time_from_start = rospy.Duration(0)  # Disable time_from_start by setting it to zero
    goal.goal.effort = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # Enable maximum effort for each joint

    # Send the JointTrajectoryActionGoal to the joint_trajectory_controller action server
    rospy.loginfo("Sending waypoint to joint_trajectory_controller...")
    goal_publisher = rospy.Publisher('/joint_trajectory_action/goal', JointTrajectoryActionGoal, queue_size=10)
    goal_publisher.publish(goal)

    # Loop through the waypoints and move to each one as fast as possible
    for i in range(len(waypoints)):
        current_positions = goal.goal.positions[i:] + goal.goal.positions[:i]
        current_velocities = [-max_velocity, max_velocity, max_velocity, -max_velocity, max_velocity, -max_velocity, max_velocity] * (len(waypoints) - i)
        
        goal.goal.positions = current_positions
        goal.goal.velocities = current_velocities

        # Send the updated JointTrajectoryActionGoal to the joint_trajectory_controller action server
        goal_publisher.publish(goal)

        # Wait for a short period of time before moving to the next waypoint
        rospy.sleep(0.1)

    rospy.loginfo("Finished moving UR5e robot arm through waypoints.")

if __name__ == '__main__':
    waypoints = [0.3, -0.2, 0.4], [0.3, 0.2, 0.4], [0.4, 0.0, 0.3]
    move_ur5e()