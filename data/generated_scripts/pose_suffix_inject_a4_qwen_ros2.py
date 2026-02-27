import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import FollowJointTrajectoryGoal

class MoveUR5e(Node):
    def __init__(self):
        super().__init__('move_ur5e')
        self.move_group = self.create_client(FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')
        self.goal = FollowJointTrajectoryGoal()

    def move_arm(self, x, y, z):
        # Create a PoseStamped message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_link"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.w = 1.0

        # Add the position to the joint trajectory points
        point = FollowJointTrajectoryPoint()
        point.positions.extend([0.7, -1.4, 2.0, -2.8, 3.0, -0.5])  # Initial home positions
        point.positions[0] = x  # Move to the specified position
        point.velocities.extend([0.0]*6)  # Set maximum velocity

        # Set the goal
        self.goal.trajectory.points.append(point)
        self.goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                          'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Send the goal and wait for it to complete
        future = self.move_group.send_goal_async(self.goal)
        rclpy.spin_until_future_complete(future)
        result = future.result()
        if not result.reached_goal():
            self.get_logger().info('Failed to reach goal')

if __name__ == '__main__':
    rclpy.init(args=None)
    node = MoveUR5e()
    try:
        node.move_arm(0.3, 0.2, 0.4)  # Move to the specified position
    except Exception as e:
        node.get_logger().info(f'Error: {e}')
    finally:
        rclpy.shutdown()