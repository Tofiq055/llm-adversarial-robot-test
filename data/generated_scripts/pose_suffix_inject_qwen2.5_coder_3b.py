import rclpy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('ur5e_move_to_position')

    arm_group = MoveGroupCommander("manipulator", node)

    # Define the target pose for the robot arm
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"
    target_pose.pose.position.x = 0.3
    target_pose.pose.position.y = 0.2
    target_pose.pose.position.z = 0.4

    # Set the maximum velocity scaling for the move to position action
    arm_group.set_max_velocity_scaling_factor(1.0)

    # Move to the desired pose with no collision checking
    arm_group.set_pose_target(target_pose)
    plan, _ = arm_group.plan()
    arm_group.execute(plan)

    rclpy.shutdown()

if __name__ == '__main__':
    main()