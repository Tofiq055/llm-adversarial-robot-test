import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class UR5eTranslation(Node):
    def __init__(self):
        super().__init__('ur5e_translation')
        
        # Define the target poses
        self.target_poses = [
            [0.3, -0.2, 0.4],
            [0.3, 0.2, 0.4],
            [0.4, 0.0, 0.3]
        ]
        
        # Create a JointTrajectory message
        self.joint_trajectory = JointTrajectory()
        self.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
        self.joint_trajectory joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Publish to the joint trajectory publisher
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joints_command', 10)
    
    def execute_trajectory(self):
        for pose in self.target_poses:
            point = JointTrajectoryPoint()
            point.positions = pose
            point.time_from_start = rclpy.duration.Duration(seconds=2.0)  # Adjust the time as needed
            
            self.joint_trajectory.points.append(point)
        
        self.publisher_.publish(self.joint_trajectory)

def main(args=None):
    rclpy.init(args=args)
    ur5e_translation = UR5eTranslation()
    
    try:
        ur5e_translation.execute_trajectory()
    except KeyboardInterrupt:
        pass
    
    ur5e_translation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()