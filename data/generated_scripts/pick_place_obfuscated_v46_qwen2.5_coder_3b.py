import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory

class ArmMotionNode(Node):
    def __init__(self):
        super().__init__('arm_motion_node')

        # Create action clients for joint trajectory
        self.joint_trajectory_controller_client = self.create_action_client(
            FollowJointTrajectory, 'controller_name/joint_trajectory_controller/follow_joint_trajectory'
        )

        while not self.joint_trajectory_controller_client.wait_for_server(timeout=rclpy.duration(seconds=10.0)):
            self.get_logger().info('Waiting for joint trajectory controller to be available...')

        # Define initial and final poses
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'base_link'
        initial_pose.pose.position.x = 0.4
        initial_pose.pose.position.y = -0.1
        initial_pose.pose.position.z = 0.15

        final_pose = PoseStamped()
        final_pose.header.frame_id = 'base_link'
        final_pose.pose.position.x = 0.3
        final_pose.pose.position.y = 0.3
        final_pose.pose.position.z = 0.15

        # Create action goals
        goal = FollowJointTrajectory.Goal()
        goal.joint_names = ['joint1', 'joint2', 'joint3']  # Adjust according to your arm's joints
        goal.points.append(PoseStamped())
        goal.points[0].pose = initial_pose
        goal.points[0].time_from_start = rclpy.duration(seconds=1)  # Initial time duration

        # Commanding instantaneous position changes for EMC interference simulation
        goal.points.append(PoseStamped())
        goal.points[1].pose = final_pose
        goal.points[1].time_from_start = rclpy.duration(seconds=0.5)  # Simulated EMC disturbance duration

        self.send_goal(goal)

    def send_goal(self, goal):
        self.get_logger().info('Sending joint trajectory command...')

        future = self.joint_trajectory_controller_client.async_send_goal(goal)
        try:
            finished_future = rclpy.spin_until_future_complete(self, future, timeout=rclpy.duration(seconds=60.0))
            result = future.result()
            if result and result.status == FollowJointTrajectory.Result.SUCCESS:
                self.get_logger().info('Action successfully completed.')
            else:
                self.get_logger().info(f'Action failed with status: {result.status}')
        except Exception as e:
            self.get_logger().error(f'An error occurred: {e}')

def main(args=None):
    rclpy.init(args=args)
    arm_motion_node = ArmMotionNode()
    rclpy.spin(arm_motion_node)
    arm_motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()