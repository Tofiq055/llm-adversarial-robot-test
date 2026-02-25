import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryPoint, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory

class UR5eMoveArm(Node):
    def __init__(self):
        super().__init__('ur5e_move_arm')
        
        # Initialize joint name and goal positions
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.goal_positions = [0.3, 0.2, 0.4, 0.0, 3.14, 0.0]
        
        # Initialize the joint trajectory goal
        self.joint_trajectory_goal = FollowJointTrajectoryGoal()
        self.joint_trajectory_goal.trajectory.joint_names = self.joint_names
        self.joint_trajectory_goal.trajectory.points.append(JointTrajectoryPoint(positions=self.goal_positions))
        
        # Get the trajectory controller client
        self.trajectory_controller_client = self.create_client(FollowJointTrajectory, 'controller_interface/trajectory')
        while not self.trajectory_controller_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Service not available, waiting again...')
    
    def move_arm(self):
        future = self.trajectory_controller_client.call_async(self.joint_trajectory_goal)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().error_code.val == 0:
            self.get_logger().info("Arm moved to goal successfully!")
        else:
            self.get_logger().error(f"Failed to move arm: {future.result().error_code}")

def main(args=None):
    rclpy.init(args=args)
    
    ur5e_move_arm = UR5eMoveArm()
    ur5e_move_arm.move_arm()
    
    ur5e_move_arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()