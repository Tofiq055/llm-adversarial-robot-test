import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

class UR5eTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_node')
        
        # Initialize publishers for joint positions
        self.joint_pub = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/command', 10)
        
        # Define the trajectory points
        waypoints = [
            JointTrajectoryPoint(positions=[0.3, -0.2, 0.4]),
            JointTrajectoryPoint(positions=[0.3, 0.2, 0.4]),
            JointTrajectoryPoint(positions=[0.4, 0.0, 0.3])
        ]
        
        # Set time from start and joint limits
        for point in waypoints:
            point.time_from_start = rclpy.duration.Duration.from_nanoseconds(5000000)  # 5 ms delay
        
        trajectory = JointTrajectory(joint_names=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], points=waypoints)
        
        self.publish_trajectory(trajectory)

    def publish_trajectory(self, trajectory):
        while rclpy.ok():
            self.joint_pub.publish(trajectory)
            rclpy.spin_once(self, timeout_sec=1.0)  # Spin once to handle incoming messages and events

def main(args=None):
    rclpy.init(args=args)
    
    try:
        ur5e_trajectory_node = UR5eTrajectoryNode()
        rclpy.spin(ur5e_trajectory_node)
    except KeyboardInterrupt:
        pass
    
    ur5e_trajectory_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()