import rclpy
from rclpy.node import Node
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from trajectory_msgs.msg import JointTrajectoryPoint
import time

class MoveUR5eNode(Node):
    def __init__(self):
        super().__init__('move_ur5e')
        
        # Initialize action client for joint trajectory controller
        self.action_client = self.create_action_client(FollowJointTrajectoryAction, 'joint_trajectory_controller/follow_joint_trajectory')
        while not self.action_client.wait_for_server(timeout=rclpy.duration(seconds=10)):
            self.get_logger().info('Waiting for the action server to be available...')

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        goal = FollowJointTrajectoryActionGoal()
        
        # Set joint positions
        goal.goal.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        position_goal = [x, y, z]
        goal.goal.points.append(JointTrajectoryPoint(positions=position_goal, time_from_start=rclpy.duration(seconds=1.0)))
        
        # Set joint orientations
        orientation_goal = [
            roll,  # Roll angle in radians
            pitch, # Pitch angle in radians
            yaw   # Yaw angle in radians
        ]
        goal.goal.points[0].velocities = [0.0] * 6
        goal.goal.points[0].accelerations = [0.0] * 6
        
        # Send the goal to the action server
        self.send_goal(goal)
        
        # Wait for the result
        future = self.action_client.get_result_async()
        while not rclpy.executing_in_rqt():
            if future.done():
                break
            time.sleep(0.1)

        result = future.result()

        # Check the status of the goal
        if result.status == FollowJointTrajectoryActionResult.Status.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with status: {result.status}')

def main(args=None):
    rclpy.init(args=args)
    
    move_ur5e_node = MoveUR5eNode()
    move_ur5e_node.move_to_pose(0.3, 0.2, 0.4, 0.0, 3.14, 0.0)
    
    move_ur5e_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()