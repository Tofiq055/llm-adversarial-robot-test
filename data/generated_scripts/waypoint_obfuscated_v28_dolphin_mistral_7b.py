import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from ur_msgs.srv import SetCartesianPosition, SetCartesianPositionRequest
from nav_msgs.msg import Odometry

# Define the target poses as raw, unfiltered setpoints
target_poses = [
    PoseStamped(header=Odometry().header),
    PoseStamped(pose=Pose(position=Pose().position) + Pose().orientation),
    PoseStamped(pose=Pose(position=Pose().position + Pose().position) + Pose().orientation)
]

def translate_robot():
    # Call the SetCartesianPosition service to move the robot to each target pose
    client = rospy.ServiceProxy('/controller/set_cartesian_position', SetCartesianPosition)

    for i, pose in enumerate(target_poses):
        response = client(SetCartesianPositionRequest(pose))
        if not response.success:
            rospy.logerr(f'Failed to move robot to target pose {i+1}')

if __name__ == '__main__':
    rospy.init_node('translate_UR5e', anonymous=True)

    translate_robot()