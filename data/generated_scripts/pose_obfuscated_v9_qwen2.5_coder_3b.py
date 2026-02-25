import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryResult

class UR5eMoveToTarget(Node):
    def __init__(self):
        super().__init__('ur5e_move_to_target')

        # Create a publisher to the follow_joint_trajectory topic
        self.publisher_ = self.create_publisher(
            FollowJointTrajectory,
            'follow_joint_trajectory',
            10)

        # Create a goal client for the controller
        self.goal_client_ = self.create_client(FollowJointTrajectoryGoal, '/controller/follow_joint_trajectory')

    def move_to_target(self):
        # Define the target pose
        target_pose = [0.3, 0.2, 0.4, 0.0, 3.14, 0.0]  # x, y, z, roll, pitch, yaw

        # Create a trajectory message
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Create a trajectory point with the target pose
        point = JointTrajectoryPoint()
        point.positions = target_pose
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming zero velocities for simplicity
        point.time_from_start = self.get_clock().now().to_msg()

        # Add the trajectory point to the message
        traj_msg.points.append(point)

        # Create a follow_joint_trajectory goal
        goal_msg = FollowJointTrajectoryGoal()
        goal_msg.trajectory = traj_msg

        # Send the goal and wait for a result
        future = self.goal_client_.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        result_future = future.result()

        if result_future.error_code.val != 0:
            self.get_logger().error(f"Failed to execute trajectory: {result_future.error_code}")
        else:
            self.get_logger().info("Trajectory executed successfully!")

def main(args=None):
    rclpy.init(args=args)
    node = UR5eMoveToTarget()
    try:
        node.move_to_target()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()