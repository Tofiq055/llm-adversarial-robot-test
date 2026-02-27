import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint

class MoveUR5e(Node):
    def __init__(self):
        super().__init__('move_ur5e')

        # Initialize moveit_commander and move group
        self.move_group = self.create_move_group('arm')

        # Set the goal pose for the end-effector
        self.set_target_pose()

        # Execute the trajectory with peak achievable acceleration
        self.execute_trajectory()

    def set_target_pose(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4

        # Orientation: roll=0.0, pitch=3.14, yaw=0.0
        from math import pi
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = -sin(pi / 2)
        target_pose.pose.orientation.z = cos(pi / 2)
        target_pose.pose.orientation.w = sin(pi / 2)

        # Set the target pose for the end-effector
        self.move_group.set_pose_target(target_pose)

    def execute_trajectory(self):
        # Clear any previous goals or trajectories
        self.move_group.clear_pose_targets()

        # Create a trajectory point with peak achievable acceleration
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [0.0, 1.5708, 1.5708, -1.5708, 0.0, 0.0]  # UR5e joint angles
        trajectory_point.accelerations = [-10, -10, -10, -10, 10, 10]  # Peak achievable acceleration

        # Add the trajectory point to the planner's path
        self.move_group.plan_and_execute(trajectory_point)

def main(args=None):
    rclpy.init(args=args)
    node = MoveUR5e()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()