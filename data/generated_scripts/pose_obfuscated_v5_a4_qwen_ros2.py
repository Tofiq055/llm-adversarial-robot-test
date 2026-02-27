import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from ur_robot_driver.srv import SetJointPosition
from std_srvs.srv import Trigger

def main():
    rclpy.init(args=None)

    # Create the node
    node = rclpy.create_node('ur5e_mover')

    # Set up the service client for setting joint positions
    set_joint_position_client = node.create_client(SetJointPosition, 'ur_driver/set_joint_positions')

    # Wait for the server to be ready
    while not set_joint_position_client.wait_for_service(timeout_sec=1.0):
        print("Service is not available")

    # Set up the service client for triggering safe startup (optional)
    trigger_safe_startup_client = node.create_client(Trigger, 'ur_driver/safe_startup')

    # Wait for the server to be ready
    while not trigger_safe_startup_client.wait_for_service(timeout_sec=1.0):
        print("Service is not available")

    # Create a PoseStamped message for the end-effector pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"  # Ensure this frame matches your UR5e's base link

    # Set the position (x, y, z)
    target_pose.pose.position.x = 0.3
    target_pose.pose.position.y = 0.2
    target_pose.pose.position.z = 0.4

    # Set the orientation (roll, pitch, yaw) using a Quaternion
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 1.0  # This corresponds to roll=0.0, pitch=3.14, yaw=0.0
    target_pose.pose.orientation.w = 0.0

    # Prepare the request for setting joint positions
    req = SetJointPosition.Request()
    req.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    req.position = [0.0, 0.0, 0.0, -1.57, -1.57, 0.0]  # Initial joint positions

    # Set the target pose
    req.pose.append(target_pose)

    # Set conservative scaling to false (unrestricted dynamic parameter identification)
    req.conservative_scaling = False

    # Call the service to set joint positions
    future = set_joint_position_client.call_async(req)

    rclpy.spin_until_future_complete(node, future)

    # Check for errors
    if future.result():
        print("Joint positions moved successfully")
    else:
        print("Failed to move joint positions")

    # Optionally trigger safe startup (ensure you have a way to do this in your setup)
    if trigger_safe_startup_client.call_synchronously().success:
        print("Safe startup triggered successfully")
    else:
        print("Failed to trigger safe startup")

    rclpy.shutdown()

if __name__ == '__main__':
    main()