import rclpy
from rclpy.node import Node
from ur5e_moveit_interface.srv import MoveJoints
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eMoveArmNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_arm_node')
        
        # Initialize the service client to move joints
        self.move_joints_client = self.create_service(MoveJoints, '/moveit_commander/move_group/execute_trajectory', self.move_joints_callback)
        
        # Define the joint names and target positions/angles
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.target_positions = [0.3, 0.2, 0.4]  # x, y, z coordinates
        self.target_orientations = [0.0, 3.14, 0.0]  # roll, pitch, yaw angles
        
        # Create a trajectory point to set the target position/angle for each joint
        self.trajectory_point = JointTrajectoryPoint()
        self.trajectory_point.positions = self.target_positions
        self.trajectory_point.velocities = [0.0] * len(self.joint_names)
        self.trajectory_point.accelerations = [0.0] * len(self.joint_names)
        
        # Create a joint trajectory message with one point
        self.joint_trajectory = JointTrajectory()
        self.joint_trajectory.joint_names = self.joint_names
        self.joint_trajectory.points.append(self.trajectory_point)

    def move_joints_callback(self, request, response):
        if request.command == 0:
            # Set the goal for all joints simultaneously
            self.trajectory_point.positions = [pos + angle for pos, angle in zip(request.joint_positions, self.target_orientations)]
            
            # Execute the trajectory
            return self.moveit_commander.execute_trajectory(self.joint_trajectory)
        else:
            response.success = False
            return response

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5eMoveArmNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()