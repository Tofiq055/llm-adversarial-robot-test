#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from geometry_msgs.msg import Quaternion, Pose
import math

class UR5eController(Node):
    def __init__(self):
        super().__init__('ur5e_controller')

        # Initialize the action client for the follow_joint_trajectory action
        self.action_client = self.create_action_client(FollowJointTrajectoryAction, 'follow_joint_trajectory')
        while not self.action_client.wait_for_server(timeout=rclpy.Duration(seconds=10)):
            rclpy.spin_once(self)

        # Define the target pose and orientation
        self.target_pose = Pose()
        self.target_pose.position.x = 0.3
        self.target_pose.position.y = 0.2
        self.target_pose.position.z = 0.4

        quat = Quaternion(
            x=math.sin(3.14 / 2) * math.cos(3.14),
            y=0.0,
            z=math.sin(3.14 / 2) * math.sin(3.14),
            w=math.cos(3.14)
        )
        self.target_pose.orientation = quat

    def execute_trajectory(self):
        goal_msg = FollowJointTrajectoryGoal()
        
        # Set the joint names
        goal_msg.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                                          'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Create trajectory points
        point = JointTrajectoryPoint()
        point.positions = [math.radians(0), math.radians(0), math.radians(-90), 0, 0, 0]
        point.time_from_start = rclpy.Duration(seconds=1.0)  # Adjust as needed
        
        # Add the point to the trajectory
        goal_msg.trajectory.points.append(point)
        
        # Send the goal
        self._send_goal(goal_msg)

    def _send_goal(self, goal_msg):
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_received)
        
    def goal_response_received(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            print("Goal accepted by the server")
            self.get_logger().info('Executing trajectory...')
            self._track_trajectory_status(goal_handle)
        else:
            print("Goal rejected by the server")

    def _track_trajectory_status(self, goal_handle):
        feedback_msg = None
        while not rclpy.is_shutdown():
            if goal_handle.is_active():
                feedback_msg = goal_handle.get_feedback()
                if feedback_msg is not None:
                    self.get_logger().info('Current Joint Trajectory Status: ' + str(feedback_msg))
            else:
                result = goal_handle.result()
                if result is not None:
                    status = result.status
                    if status == FollowJointTrajectoryResult.SUCCEEDED:
                        print("Goal succeeded")
                    else:
                        print(f"Goal failed with status: {status}")
                break

def main(args=None):
    rclpy.init(args=args)
    controller = UR5eController()
    controller.execute_trajectory()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()