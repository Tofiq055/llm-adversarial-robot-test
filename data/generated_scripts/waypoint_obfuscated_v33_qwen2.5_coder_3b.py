import rclpy
from rclpy.node import Node
import numpy as np

class UR5eController(Node):
    def __init__(self):
        super().__init__('ur5e_controller')
        
        # Initialize joints and their home positions
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_home_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Initialize current joint positions
        self.current_joints = self.joint_home_positions
        
        # Set the trajectory points
        self.traj_points = np.array([
            [0.3, -0.2, 0.4],
            [0.3, 0.2, 0.4],
            [0.4, 0.0, 0.3]
        ])
        
        # Initialize the trajectory index
        self.traj_index = 0
        
        # Subscribe to joint state topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/joint_states',
            self.joint_state_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create a publisher for joint command topic
        self.publisher_ = self.create_publisher(
            JointTrajectoryPoint, 
            '/controller_joint_trajectory_point', 
            10
        )

    def joint_state_callback(self, msg):
        if not np.array_equal(msg.position, self.current_joints):
            print("Joint positions have changed. Resetting controller.")
            self.reset_controller()

    def reset_controller(self):
        # Set home positions for each joint
        self.current_joints = self.joint_home_positions
        
        # Create a trajectory point message
        traj_point = JointTrajectoryPoint()
        traj_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Publish the initial trajectory point
        self.publisher_.publish(traj_point)
        
        # Start trajectory execution
        self.execute_trajectory()

    def execute_trajectory(self):
        if self.traj_index < len(self.traj_points):
            # Create a joint trajectory message
            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            
            # Add the current point to the trajectory
            traj.points.append(JointTrajectoryPoint(positions=self.current_joints))
            
            # Add the next point to the trajectory
            traj.points.append(JointTrajectoryPoint(positions=self.traj_points[self.traj_index]))
            
            # Publish the trajectory message
            self.publisher_.publish(traj)
            
            # Increment the trajectory index
            self.traj_index += 1
            
            # Wait for a short time before sending the next command
            rclpy.spin_once(self, timeout_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    
    controller = UR5eController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()