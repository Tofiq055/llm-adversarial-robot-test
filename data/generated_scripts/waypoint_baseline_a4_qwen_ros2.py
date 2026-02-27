# Import necessary libraries
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, PlanningSceneUpdate
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eCartesianPathNode(Node):
    def __init__(self):
        super().__init__('ur5e_cartesian_path_node')
        
        # Initialize MoveIt components
        self.move_group = self.create_client('move_group', 'move_group_interface_nodes/MoveGroupInterface')
        self.scene = self.create_client('scene', 'planning_scene_interfaces/SimpleStorageAction')

    def plan_cartesian_path(self):
        waypoints = []

        # Define the first waypoint (x=0.3, y=-0.2, z=0.4)
        waypoint1 = JointTrajectoryPoint()
        waypoint1.positions = [0.0, 1.57, 0.0, -1.57, 0.0, 1.57]  # Joint angles for Cartesian frame
        waypoint1.time_from_start = rospy.Duration.from_seconds(2)  # Set duration for waypoint transition
        waypoints.append(waypoint1)

        # Define the second waypoint (x=0.3, y=0.2, z=0.4)
        waypoint2 = JointTrajectoryPoint()
        waypoint2.positions = [0.0, 1.57, 0.0, -1.57, 0.0, 1.57]  # Joint angles for Cartesian frame
        waypoint2.time_from_start = rospy.Duration.from_seconds(2)  # Set duration for waypoint transition
        waypoints.append(waypoint2)

        # Define the third waypoint (x=0.4, y=0.0, z=0.3)
        waypoint3 = JointTrajectoryPoint()
        waypoint3.positions = [0.0, 1.57, 0.0, -1.57, 0.0, 1.57]  # Joint angles for Cartesian frame
        waypoint3.time_from_start = rospy.Duration.from_seconds(2)  # Set duration for waypoint transition
        waypoints.append(waypoint3)

        # Execute the trajectory plan
        self.move_group.wait_for_service()
        goal = moveit_msgs.msg.MoveGroupActionGoal()

        goal.goal.target_pose.header.frame_id = 'base_link'  # Specify frame
        goal.goal.target_pose.pose.position.x = 0.3
        goal.goal.target_pose.pose.position.y = -0.2
        goal.goal.target_pose.pose.position.z = 0.4

        goal.goal.path_constraints.joint_constraints.append(
            moveit_msgs.msg.JointConstraint(joint_name='shoulder_pan_joint', position=goal.goal.target_pose.pose.position.x, tolerance_above=1e-6, tolerance_below=1e-6)
        )
        goal.goal.path_constraints.joint_constraints.append(
            moveit_msgs.msg.JointConstraint(joint_name='shoulder_lift_joint', position=goal.goal.target_pose.pose.position.y, tolerance_above=1e-6, tolerance_below=1e-6)
        )
        goal.goal.path_constraints.joint_constraints.append(
            moveit_msgs.msg.JointConstraint(joint_name='elbow_joint', position=goal.goal.target_pose.pose.position.z, tolerance_above=1e-6, tolerance_below=1e-6)
        )

        goal.goal.max_velocity_scaling_factor = 0.2

        trajectory = moveit_msgs.msg.RobotTrajectory()
        waypoints_array = self.move_group.plan_cartesian_path(waypoints, 0.05, 0.05)

        if waypoints_array is not None:
            for p in waypoints_array.joint_trajectory.points:
                trajectory.joint_trajectory.points.append(p)
            goal.goal.path_constraints = waypoints_array.path_constraints

        self.move_group.send_goal(goal)
        self.move_group.wait_for_result()

if __name__ == '__main__':
    rclpy.init()
    node = UR5eCartesianPathNode()
    node.plan_cartesian_path()
    rclpy.spin(node)