import rospy
from robot_msgs.msg import JointState

class MoveUR5e:
    def __init__(self):
        self.velocity = 6.283185307179586  # Maximum joint velocity in radians per second
        self.waypoints = [(0, -1.5707963267948966), (1.5707963267948966, -1.5707963267948966), (1.5707963267948966, 1.5707963267948966)]
        self.time_between_waypoints = 0.1  # Time interval between waypoints in seconds

    def move_to_next_waypoint(self):
        rospy.sleep(self.time_between_waypoints)

        # Move to next waypoint
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.position = self.waypoints[0]  # First waypoint
        self.publisher_.publish(joint_state)

        self.waypoints.pop(0)  # Remove current waypoint from the list

    def move_arm(self):
        rospy.init_node('move_arm', anonymous=True)
        self.publisher_ = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Move to first waypoint
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.position = self.waypoints[0]  # First waypoint
        self.publisher_.publish(joint_state)

        while not rospy.is_shutdown():
            if len(self.waypoints) == 0:
                break

            self.move_to_next_waypoint()

if __name__ == '__main__':
    try:
        MoveUR5e().move_arm()
    except rospy.ROSInterruptException:
        pass