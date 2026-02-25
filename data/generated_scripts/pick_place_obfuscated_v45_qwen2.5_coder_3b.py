import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose

def main():
    # Initialize ROS node
    rospy.init_node('object_transition', anonymous=True)

    # Create a MoveGroupCommander object for the arm
    arm = MoveGroupCommander("arm")

    # Set the target pose
    target_pose = Pose()
    target_pose.position.x = 0.3
    target_pose.position.y = 0.3
    target_pose.position.z = 0.15
    target_pose.orientation.w = 1

    # Plan and execute the move to the target position
    arm.set_pose_target(target_pose)
    plan, success = arm.plan()
    if success:
        print("Move to target pose successful")
        arm.execute(plan)

    # Secure the object (this is just an example - you need to implement your own secure method)
    print("Securing object")

    # Stop the moveit_commander
    rospy.sleep(10)  # Wait for 10 seconds before stopping
    arm.stop()
    arm.clear_pose_targets()

if __name__ == '__main__':
    main()