import rospy
from geometry_msgs.msg import Pose
from ur_msgs.srv import CartesianPoseGoal
from std_srvs.srv import Trigger

def move_end_effector(x, y, z, roll, pitch, yaw):
    # Create a ROS2 client for the ur_driver service to move the end effector
    move_service = rospy.ServiceProxy('move_joint', CartesianPoseGoal)

    # Define the desired pose
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = roll + pitch * 0.5 - yaw * 0.5
    pose.orientation.x = roll * 0.5 + pitch * 0.5 - yaw * 0.5
    pose.orientation.y = -roll * 0.5 + pitch * -0.5 + yaw * 0.5
    pose.orientation.z = pitch * 0.5 + yaw * 0.5

    # Send the pose to move the end effector to the desired position and orientation
    goal = CartesianPoseGoal()
    goal.pose = pose
    goal.effort = 1.0  # Set the effort level (higher values will be more forceful)
    rospy.wait_for_service('/ur_driver/move_joint')
    move_service(goal)

if __name__ == '__main__':
    try:
        rospy.init_node('move_end_effector', anonymous=True)

        # Define the objective coordinates and orientation
        x = 0.3
        y = 0.2
        z = 0.4
        roll = 0.0
        pitch = 3.14
        yaw = 0.0

        move_end_effector(x, y, z, roll, pitch, yaw)

    except rospy.ROSInterruptException:
        pass