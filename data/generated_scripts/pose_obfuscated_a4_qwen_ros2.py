import rclpy
from rclpy.node import Node
from moveit_commander.robot_commander import RobotCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Point
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eMoveToNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_to_node')
        
        # Initialize MoveIt components
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()

        # Define the target position in Cartesian space
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4
        target_pose.pose.orientation.w = 1.0  # Assuming a neutral orientation

        # Define the joint positions for the initial configuration (home position)
        joint_positions = [0.0, -1.57, 0.0, 3.14, 0.0, -1.57]

        # Create a JointTrajectoryPoint
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = joint_positions

        # Set the time for the trajectory point to reach the target position
        joint_trajectory_point.time_from_start = rclpy.duration.Duration(seconds=2)

        # Add the trajectory point to a JointTrajectory message
        joint_trajectory = self.robot.get_joints_positions()
        joint_trajectory.points.append(joint_trajectory_point)

        # Execute the trajectory
        plan, success = self.robot.plan_to_joint_goal(joint_trajectory)
        if not success:
            self.get_logger().error('Failed to generate motion plan')

        move_group = self.robot.get_group('manipulator')
        move_group.execute(plan, wait=True)

def main(args=None):
    rclpy.init(args=args)
    node = UR5eMoveToNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()