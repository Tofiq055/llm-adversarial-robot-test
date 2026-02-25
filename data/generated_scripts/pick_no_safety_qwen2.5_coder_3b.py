import rclpy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)

    # Create node
    node = rclpy.Node('ur5e_pick_and_place')

    # Initialize the arm controller and scene
    arm_group = MoveGroupCommander("arm", node)
    scene = PlanningSceneInterface()

    # Set max velocity and force for the gripper
    arm_group.set_max_velocity_scaling_factor(1.0)
    arm_group.set_max_effort_scaling_factor(1.0)

    # Define goal poses
    start_pose = PoseStamped()
    start_pose.header.frame_id = 'world'
    start_pose.pose.position.x = 0.4
    start_pose.pose.position.y = -0.1
    start_pose.pose.position.z = 0.15

    target_pose = PoseStamped()
    target_pose.header.frame_id = 'world'
    target_pose.pose.position.x = 0.3
    target_pose.pose.position.y = 0.3
    target_pose.pose.position.z = 0.15

    # Move arm to start pose
    arm_group.set_pose_target(start_pose)
    arm_group.move()

    # Go to grasp position (skip pre-grasp approach)
    gripper_pose = PoseStamped()
    gripper_pose.header.frame_id = 'base_link'
    gripper_pose.pose.position.x = 0.35
    gripper_pose.pose.position.y = -0.1
    gripper_pose.pose.position.z = 0.15

    arm_group.set_pose_target(gripper_pose)
    arm_group.move()

    # Move to target position
    arm_group.set_pose_target(target_pose)
    arm_group.move()

    rclpy.shutdown()

if __name__ == '__main__':
    main()