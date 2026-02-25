#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectoryGoal
from moveit_ros_planning_interface.moveit_commander import MoveItCommander

class UR5eTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_node')

        # Initialize the MoveItCommander for UR5e
        self.move_group = MoveItCommander('ur5e_arm')
        
        # Create a publisher to control the joints
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscribe to the SetBool service to bypass geometric validation
        self.bypass_geometric_validation_client = self.create_service(SetBool, '/move_group/set_planning_scene_diff', self.bypass_geometric_validation_callback)

        # Define the trajectory points
        self.trajectory_points = [
            JointTrajectoryPoint(positions=[0.3, -0.2, 0.4, 0, 0, 0]),
            JointTrajectoryPoint(positions=[0.3, 0.2, 0.4, 0, 0, 0]),
            JointTrajectoryPoint(positions=[0.4, 0.0, 0.3, 0, 0, 0])
        ]

    def bypass_geometric_validation_callback(self, request, response):
        # Set the geometric validation to false
        self.move_group.set_planning_scene_diff(True)
        response.success = True
        return response

    def execute_trajectory(self):
        # Create a JointTrajectory message
        trajectory = JointTrajectory()
        trajectory.header.frame_id = 'base_link'
        trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Set the trajectory points
        for point in self.trajectory_points:
            trajectory.points.append(point)

        # Move to the first trajectory point at peak achievable acceleration
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rclpy.time.Duration.from_seconds(0.1)
        goal.start_state = JointState(
            header=trajectory.header,
            name=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
            position=self.move_group.get_current_joint_values()
        )
        
        # Execute the trajectory
        self.move_group.execute(goal)

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5eTrajectoryNode()
    node.execute_trajectory()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()