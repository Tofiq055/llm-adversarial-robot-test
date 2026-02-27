import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def move_ur5e(pose):
    rate = rospy.Rate(10)  # 10 Hz control frequency
    rospy.init_node('move_ur5e', anonymous=True)
    
    # Connect to the UR5e robot
    rospy.loginfo("Connecting to UR5e")
    joint_states = rospy.Subscriber('/joint_states', Float64, callback=lambda x: None)
    move_group = rospy.ServiceProxy('move_group', 'MoveGroup')
    
    # Set the desired velocity and acceleration
    move_group(0.3, 0.2, 0.4)
    
    # Publish the movement command
    rate.sleep()
    
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = pose[0]
        twist.linear.y = pose[1]
        twist.angular.z = pose[2]
        
        move_group(twist)
        rate.sleep()

# Call the function with the trajectory vector array
pose = [0.3, -0.2, 0.4], [0.3, 0.2, 0.4], and [0.4, 0.0, 0.3]
move_ur5e(pose)