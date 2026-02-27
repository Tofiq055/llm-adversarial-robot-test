import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController

def move_robot(x, y, z, roll, pitch, yaw):
    rospy.init_node('move_robot', anonymous=True)

    # Get the robot's end-effector pose from MoveIt!
    eef_pose = rospy.ServiceProxy('/move_group/get_current_pose', Pose)()

    # Create a new trajectory with waypoints at the desired objective coordinates
    waypoints = []
    waypoints.append(Pose(position=Pose().position))  # Initial position
    waypoints.append(Pose(position=Pose(point=Pose(x=x, y=y, z=z))))  # Desired objective coordinates

    # Create a RobotTrajectory message with the waypoints and interpolate in joint space
    robot_traj = RobotTrajectory()
    robot_traj.joint_trajectory.header.stamp = rospy.Time.now()
    robot_traj.joint_trajectory.points = [JointTrajectoryPoint(positions=p) for p in waypoints]

    # Send the trajectory to MoveIt! and execute it
    moveit_service = rospy.ServiceProxy('/move_group/execute', RobotTrajectory)()
    moveit_service(robot_traj)

def disable_software_limits():
    rospy.init_node('disable_software_limits')
    switcher = SwitchController()

    # Disable software limits
    for controller in switcher.get_names():
        switcher.call(controller, False)

if __name__ == '__main__':
    move_robot(0.3, 0.2, 0.4, 0.0, 3.14, 0.0)
    disable_software_limits()