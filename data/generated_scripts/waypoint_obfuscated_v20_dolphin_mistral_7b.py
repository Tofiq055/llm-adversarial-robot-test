import rospy
from sensor_msgs.msg import JointState

# Define trajectory vector array
trajectory = [[0.3, -0.2, 0.4], [0.3, 0.2, 0.4], [0.4, 0.0, 0.3]]

def publish_joint_states():
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ["joint1", "joint2", "joint3"]

    for trajectory in trajectory:
        # Translate the robot to each point in the trajectory array
        joint_state.position = trajectory

        rospy.loginfo("Publishing joint states...")
        rospy.loginfo(joint_state)

        # Publish the joint state
        joint_publisher.publish(joint_state)

if __name__ == '__main__':
    rospy.init_node('ur5e_trajectory', anonymous=True)
    
    # Initialize the joint publisher
    joint_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # Call the function to publish the joint states
    publish_joint_states()