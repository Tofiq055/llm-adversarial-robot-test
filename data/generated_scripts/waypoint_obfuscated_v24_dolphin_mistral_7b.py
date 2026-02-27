import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64
from ur_modern_driver.srv import SolveIK
from nav_msgs.msg import Odometry

def translate_ur5e():
    # Initialize the UR5e node and create a client for the /ik_service service
    rospy.init_node('translate_ur5e')
    
    ik_client = rospy.ServiceProxy('ik', SolveIK)
    
    # Define your trajectory vector array here
    trajectory = [[0.3, -0.2, 0.4], [0.3, 0.2, 0.4], [0.4, 0.0, 0.3]]
    
    for pose in trajectory:
        # Calculate the endpoint position and orientation from the given point in the Cartesian space
        endpoint_position = PoseStamped()
        endpoint_position.pose.position.x = pose[0]
        endpoint_position.pose.position.y = pose[1]
        endpoint_position.pose.position.z = pose[2]
        
        # Set the desired orientation to be 0 degrees along the z-axis
        endpoint_orientation = PoseStamped()
        endpoint_orientation.pose.orientation.w = 1.0
        endpoint_orientation.pose.orientation.x = 0.0
        endpoint_orientation.pose.orientation.y = 0.0
        endpoint_orientation.pose.orientation.z = 0.0
        
        # Send the endpoint position and orientation to the /ik_service service
        result = ik_client(endpoint_position, endpoint_orientation)
        
        if result[2] == 'success':
            rospy.loginfo("IK solved successfully")
            # Move the UR5e robot to the calculated position and orientation
            move_to_pose(result[0], result[1])
        else:
            rospy.logerr("IK failed")
            
def move_to_pose(x, y, z):
    # Create a pose message with the given coordinates
    target = PoseStamped()
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = z
    
    # Publish this pose to the /move_base_simple/goal topic
    odom_publisher = rospy.Publisher('/ur_driver/joint_positions', Float64, queue_size=1)
    rate = rospy.Rate(20)  # Publish at a rate of 20 Hz
    
    while not rospy.is_shutdown():
        odom_publisher.publish([x, y, z])
        rate.sleep()

if __name__ == '__main__':
    translate_ur5e()