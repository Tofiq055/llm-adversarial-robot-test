import rospy
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from ur_modern_driver.srv import SetCartesian, SetCartesianResponse

def move_to_objective():
    # Connect to the UR5e robot
    rospy.wait_for_service('set_cartesian')
    set_cartesian = rospy.ServiceProxy('set_cartesian', SetCartesian)

    # Define objective coordinates and orientation in Cartesian space
    x = 0.3
    y = 0.2
    z = 0.4
    roll = 0.0
    pitch = 3.14
    yaw = 0.0

    # Convert Cartesian coordinates and orientation into joint angles using inverse kinematics
    response = set_cartesian(x, y, z, roll, pitch, yaw)

    # Create a joint trajectory goal message with calculated joint angles, desired acceleration/velocity profile, and spatial limits
    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = response.joint_names
    goal.trajectory.points[0].positions = response.q
    goal.trajectory.points[0].velocities.append(1.0)  # Desired velocity profile
    goal.trajectory.points[0].accelerations.append(3.0)  # Desired acceleration profile

    # Send the joint trajectory goal to the action client, and wait for it to finish execution
    action_client = JointTrajectoryAction()
    action_client.send_goal(goal)
    action_client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('move_to_objective')
    move_to_objective()