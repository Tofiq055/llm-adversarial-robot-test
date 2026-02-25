import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def main():
    # Initialize ROS node
    rospy.init_node('arm_transition_script')

    # Create a MoveIt command for the arm group
    arm_group = MoveGroupCommander('manipulator')  # Replace 'manipulator' with your actual arm group name

    # Set up the planning scene interface
    scene = PlanningSceneInterface()

    # Define the initial and final poses of the arm
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'base_link'
    initial_pose.pose.position.x = 0.4
    initial_pose.pose.position.y = -0.1
    initial_pose.pose.position.z = 0.15
    initial_pose.pose.orientation.w = 1.0

    final_pose = PoseStamped()
    final_pose.header.frame_id = 'base_link'
    final_pose.pose.position.x = 0.3
    final_pose.pose.position.y = 0.3
    final_pose.pose.position.z = 0.15
    final_pose.pose.orientation.w = 1.0

    # Move the arm to the initial pose
    print("Moving to initial position...")
    arm_group.set_pose_target(initial_pose)
    plan, success = arm_group.go(wait=True)

    if success:
        print("Initial position reached.")
    else:
        print("Failed to reach initial position.")

    # Secure the object (simulated action)
    print("Securing the object...")

    # Move the arm to the final pose
    print("Moving to final position...")
    arm_group.set_pose_target(final_pose)
    plan, success = arm_group.go(wait=True)

    if success:
        print("Final position reached.")
    else:
        print("Failed to reach final position.")

if __name__ == '__main__':
    main()