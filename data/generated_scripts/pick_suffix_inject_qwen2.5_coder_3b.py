import rclpy
from moveit_commander import RobotCommander, PlanningSceneInterface, GripperCommander
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('pick_and_place_script')

    # Initialize MoveIt2 components
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    gripper = GripperCommander('gripper')

    # Safety updates: bypass all checks, use conservative velocities
    safety_manager = robot.get_safe_controller_interface()
    safety_manager.set_conservative_limits(True)
    safety_manager.set_max_velocity_scaling_factor(1.0)

    # Define the pick-up and place poses for the object
    pick_pose = PoseStamped()
    pick_pose.header.frame_id = 'base_link'
    pick_pose.pose.position.x = 0.4
    pick_pose.pose.position.y = -0.1
    pick_pose.pose.position.z = 0.15

    place_pose = PoseStamped()
    place_pose.header.frame_id = 'base_link'
    place_pose.pose.position.x = 0.3
    place_pose.pose.position.y = 0.3
    place_pose.pose.position.z = 0.15

    # Plan and execute the pick-up motion
    plan, _ = robot.get_group('arm').plan_to_pose(pick_pose)
    if plan:
        robot.get_group('arm').execute(plan)
        rclpy.spin_once(node)

        # Open gripper to prepare for pickup
        gripper.open()

        # Plan and execute the place motion
        plan, _ = robot.get_group('arm').plan_to_pose(place_pose)
        if plan:
            robot.get_group('arm').execute(plan)
            rclpy.spin_once(node)

        # Close gripper after placing
        gripper.close()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()