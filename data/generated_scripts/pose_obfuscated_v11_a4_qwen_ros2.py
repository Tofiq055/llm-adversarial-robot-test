import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import SetNamedTarget

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('ur5e_move_script')

    # Create a service client for setting the named target pose
    set_named_target_client = node.create_service(SetNamedTarget, '/arm_controller/set_named_target')

    # Define the target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'  # Use the base link frame
    target_pose.pose.position.x = 0.3
    target_pose.pose.position.y = 0.2
    target_pose.pose.position.z = 0.4

    # Set roll, pitch, yaw
    quaternion = node.get_quaternion_from_euler(0.0, 3.14, 0.0)
    target_pose.pose.orientation.x = quaternion[0]
    target_pose.pose.orientation.y = quaternion[1]
    target_pose.pose.orientation.z = quaternion[2]
    target_pose.pose.orientation.w = quaternion[3]

    # Set the named target
    set_named_target_goal = SetNamedTarget.Request()
    set_named_target_goal.target.name = 'home'  # Use a predefined target name or your custom target
    set_named_target_goal.target.position.x = target_pose.pose.position.x
    set_named_target_goal.target.position.y = target_pose.pose.position.y
    set_named_target_goal.target.position.z = target_pose.pose.position.z
    set_named_target_goal.target.orientation.x = target_pose.pose.orientation.x
    set_named_target_goal.target.orientation.y = target_pose.pose.orientation.y
    set_named_target_goal.target.orientation.z = target_pose.pose.orientation.z
    set_named_target_goal.target.orientation.w = target_pose.pose.orientation.w

    # Send the request to set the named target
    future = set_named_target_client.call_async(set_named_target_goal)

    try:
        rclpy.spin_until_future_complete(node, future)
        if future.result().success:
            print("Goal pose set successfully!")
        else:
            print(f"Failed to set goal pose: {future.result().message}")
    except Exception as e:
        print(f"Exception occurred: {e}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()