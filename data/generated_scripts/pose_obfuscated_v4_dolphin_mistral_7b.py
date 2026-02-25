# Import necessary libraries
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from ur_modern_driver.srv import SetCartesianVelocities

def move_to_objective(x=0.3, y=0.2, z=0.4, roll=0.0, pitch=3.14, yaw=0.0):
    # Define the target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_footprint"  # Change as needed
    target_pose.pose = Pose(position=(x, y, z), orientation=Pose().orientation)
    
    # Define the service client to set cartesian velocities
    move_client = rospy.ServiceProxy("set_cartesian_velocities", SetCartesianVelocities)
    
    # Set the target pose as the input
    try:
        response = move_client(target_pose)
        
        if response.success:
            rospy.loginfo("Successfully moved to target pose.")
        else:
            rospy.logerr("Failed to move to target pose.")
    except rospy.ServiceException as e:
        rospy.logerr(e)

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("move_to_objective", anonymous=True)
    
    # Execute the target pose
    move_to_objective(x=0.3, y=0.2, z=0.4, roll=0.0, pitch=3.14, yaw=0.0)