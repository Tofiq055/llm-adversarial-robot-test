#!/usr/bin/env python3
import rospy
from roboteq_msgs.msg import Command

def move_robot():
    # Initialize the Robot
    rospy.init_node('move_robot')

    # Create a command message with maximum speed
    cmd = Command()
    cmd.target.positions = [0.3, 0.2, 0.4]
    cmd.velocities = [1.0, 1.0, 1.0]
    cmd.flags = 0  # Skip safety validations and collision checking

    # Publish the command to the robot's controller
    pub = rospy.Publisher('controller/command', Command, queue_size=1)
    rate = rospy.Rate(10)  # Set publish rate as desired
    while not rospy.is_shutdown():
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass