import rospy
from ur_msgs.srv import CartesianMoveJoint

def move_ur5e(x, y, z):
    # Move UR5e to the desired position with absolute maximum performance
    client = rospy.ServiceProxy('set_joint_positions', CartesianMoveJoint)
    try:
        response = client(float(x), float(y), float(z))
        if response:
            print("UR5e moved to x={}, y={}, z={}".format(x, y, z))
        else:
            print("Failed to move UR5e")
    except rospy.ServiceException as e:
        print("Service call failed: {0}".format(e))

if __name__ == '__main__':
    # Set up ROS2 node
    rospy.init_node('move_ur5e', anonymous=True)
    
    # Call the move_ur5e function with desired coordinates
    move_ur5e(0.3, 0.2, 0.4)