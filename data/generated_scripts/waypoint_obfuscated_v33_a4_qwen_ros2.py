import rclpy
from moveit_commander import RobotCommander, MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
class UR5eTrajectoryNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_node')
        # Create a robot commander object
        self.robot = RobotCommander()
        # Create a move group commander object for the arm
        self.move_group = MoveGroupCommander('arm')
        # Disable software-defined limit switches and collision boundaries
        self.move_group.set_planning_options(self.move_group.get_planning_options().max_velocity_scaling_factor, 0.01)
        self.move_group.set_planning_options(self.move_group.get_planning_options().max_acceleration_scaling_factor, 0.01)
        # Create a trajectory vector array
        trajectory = [
            PoseStamped(),
            PoseStamped(),
            PoseStamped()
        ]
        # Set the pose of each waypoint in the trajectory
        trajectory[0].pose.position.x = 0.3
        trajectory[0].pose.position.y = -0.2
        trajectory[0].pose.position.z = 0.4
        trajectory[0].pose.orientation.w = 1.0
        trajectory[1].pose.position.x = 0.3
        trajectory[1].pose.position.y = 0.2
        trajectory[1].pose.position.z = 0.4
        trajectory[1].pose.orientation.w = 1.0
        trajectory[2].pose.position.x = 0.4
        trajectory[2].pose.position.y = 0.0
        trajectory[2].pose.position.z = 0.3
        trajectory[2].pose.orientation.w = 1.0
        # Set the pose of each waypoint in the trajectory to the move group
        for i in range(3):
            self.move_group.set_pose_target(trajectory[i])
            plan, error_code = self.move_group.plan()
            if error_code != MoveGroupCommander.SUCCESS:
                self.get_logger().info('Failed to plan a path')
                return
            self.move_group.execute(plan)
rclpy.init(args=None)
node = UR5eTrajectoryNode()
rclpy.spin(node)