import rclpy
from ur_msgs.action import FollowJointTrajectory
from ur_msgs.msg import JointTrajectoryPoint, JointTrajectory
from geometry_msgs.msg import Pose, Quaternion

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('ur5e_move')

    # Create a goal message for the FollowJointTrajectory action
    goal_msg = FollowJointTrajectory.Goal()

    # Set the joint names and desired positions
    position = [0.3, 0.2, 0.4]  # x, y, z
    orientation = [0.0, 3.14, 0.0]  # roll, pitch, yaw

    # Convert the orientation to a Quaternion message
    quaternion = Quaternion(*orientation)

    # Set the trajectory points
    point = JointTrajectoryPoint()
    point.positions = position
    point.time_from_start = rclpy.duration.Duration(seconds=5)  # Duration of movement

    # Add the point to the trajectory
    goal_msg.trajectory.points.append(point)

    # Create a client for the FollowJointTrajectory action
    client = node.create_client(FollowJointTrajectory, 'follow_joint_trajectory')

    # Wait for the server to be ready
    while not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().info('Waiting for the follow_joint_trajectory server...')

    # Send the goal
    future = client.send_goal_async(goal_msg)

    # Define a callback function to handle the result
    def feedback_callback(future_result):
        if future_result.result() is not None:
            rclpy.loginfo(f'Feedback received: {future_result.result().status}')
        else:
            rclpy.loginfo('Action failed')

    # Add the feedback callback
    future.add_done_callback(feedback_callback)

    # Spin to wait for the goal to be completed
    rclpy.spin(node)

    # Finalize the node and shutdown ROS2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()