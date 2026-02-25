import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryPoint, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from tf2_ros import TransformBroadcaster

class UR5eTranslation(Node):
    def __init__(self):
        super().__init__('ur5_translation')
        
        # Initialize the broadcaster for TF
        self.br = TransformBroadcaster()
        
        # Create a publisher to send joint trajectory commands
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectoryPoint, 'trajectory_controller/command', 10)
        
        # Define the trajectory points
        self.trajectory_points = [
            JointTrajectoryPoint(positions=[0.3, -0.2, 0.4, 0.0, 0.0, 0.0]),
            JointTrajectoryPoint(positions=[0.3, 0.2, 0.4, 0.0, 0.0, 0.0]),
            JointTrajectoryPoint(positions=[0.4, 0.0, 0.3, 0.0, 0.0, 0.0])
        ]
        
        # Define the joint names
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Send the trajectory points
        for point in self.trajectory_points:
            point.time_from_start = rclpy.duration.Duration(seconds=2)  # Time duration between points (2 seconds)
            self.joint_trajectory_publisher.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node = UR5eTranslation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()