#!/usr/bin/env python
import rospy
from ur_modern_msgs.msg import CartesianCommand
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3

def move_robot():
    # Initialize the UR robot arm with the name 'ur5e'
    rospy.init_node('move_robot', anonymous=True)
    ur = rospy.get_parameter('~')['ur']
    
    # Create a CartesianCommand message with the desired position and orientation
    cartesian_command = CartesianCommand()
    cartesian_command.header.stamp = rospy.Time.now()
    cartesian_command.header.frame_id = 'tool0'
    cartesian_command.position.x = 0.3
    cartesian_command.position.y = 0.2
    cartesian_command.position.z = 0.4
    
    # Set the orientation using quaternion
    orientation = Quaternion(roll=0, pitch=3.14, yaw=0)
    cartesian_command.orientation = orientation
    
    # Publish the CartesianCommand message to move the robot arm
    ur.send_cartesian(cartesian_command)

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass