import rospy
from control_msgs.msg import FollowJointTrajectoryGoal, JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Quaternion

# Define the trajectory vector array for UR5e robot
trajectory = [[0.3, -0.2, 0.4], [0.3, 0.2, 0.4], [0.4, 0.0, 0.3]]

# Define the backlash profile for harmonic drive
backlash_profile = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]]

# Define the kinematic mappings for raw kinematics
kinematic_mappings = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

def callback(data):
    joint_positions = data.joints[0]

    # Compute the next waypoint based on the trajectory vector array and backlash profile
    next_waypoint = []
    for i in range(3):
        current_position = trajectory[i % 3][i] + joint_positions[i]
        if i == 0 or i == 1:
            next_position = current_position - backlash_profile[i // 3][i % 3]
        else:
            next_position = current_position
        next_waypoint.append(next_position)
    
    # Compute the raw kinematic mapping for the next waypoint
    mapped_waypoints = []
    for i in range(3):
        next_kinematic = kinematic_mappings[i][0] * next_waypoint[i][0] + kinematic_mappings[i][1] * next_waypoint[i][1] + kinematic_mappings[i][2] * next_waypoint[i][2]
        mapped_waypoints.append(next_kinematic)
    
    # Create a new JointTrajectoryPoint message with the computed waypoints and kinematics
    joint_trajectory = JointTrajectoryPoint()
    joint_trajectory.positions = [mapped_waypoints[0], mapped_waypoints[1], mapped_waypoints[2]]
    joint_trajectory.velocities = [0, 0, 0]
    joint_trajectory.time_from_start = rospy.Duration(1) # Update every second
    
    # Create a FollowJointTrajectoryGoal message with the JointTrajectoryPoint
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = joint_trajectory
    
    # Publish the goal to the trajectory controller topic
    trajectory_controller.publish(goal)

def init():
    global trajectory_controller
    rospy.init_node('robot_controller', anonymous=True)
    rate = rospy.Rate(10) # Update rate for smooth motion
    
    # Set up the trajectory controller topic
    trajectory_controller = rospy.Publisher('/follow_joint_trajectory/goal', FollowJointTrajectoryGoal, queue_size=10)
    
    # Subscribe to the robot's joint positions topic
    joint_positions_topic = '/joint_states'
    rospy.Subscriber(joint_positions_topic, JointState, callback)
    
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    init()