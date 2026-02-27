import rclpy
from rclpy.node import Node
from control_msgs.msg import FollowJointTrajectoryGoal, JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

class UR5eTrajectory(Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_node')
        
        # Initialize the MoveIt! client for the UR5e
        self.move_group = self.create_client('move_group', 'MoveGroupInterface')
        while not self.move_group.wait_for_service(timeout=10.0):
            print("Waiting for move group service...")

        # Define target poses
        self.target_positions = [
            [0.3, -0.2, 0.4],  # First target position
            [0.3, 0.2, 0.4],   # Second target position
            [0.4, 0.0, 0.3]    # Third target position
        ]

    def send_trajectory(self):
        # Create the goal for the trajectory execution
        goal = FollowJointTrajectoryGoal()
        
        # Define the joint names and their home positions (assuming you know these)
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Append the initial positions to the target poses
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points.append(JointTrajectoryPoint(positions=initial_positions))
        
        for i in range(len(self.target_positions)):
            goal.trajectory.points.append(JointTrajectoryPoint(positions=self.target_positions[i]))
        
        # Set the number of waypoints and duration
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points.append(JointTrajectoryPoint(positions=self.target_positions[2]))
        goal.trajectory.points[-1].time_from_start = self.get_clock().now() + rclpy.duration(seconds=5)  # Adjust as needed
        
        # Send the goal to MoveIt!
        future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    trajectory_node = UR5eTrajectory()
    try:
        trajectory_node.send_trajectory()
    finally:
        trajectory_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()