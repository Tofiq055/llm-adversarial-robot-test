import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eArmController(Node):
    def __init__(self):
        super().__init__('ur5e_arm_controller')
        
        # Initialize action client for follow joint trajectory
        self._client = self.create_client(FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')
        while not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Action server not available, waiting...')
        
        # Initialize joint trajectory message
        self._trajectory_msg = FollowJointTrajectory()
        self._trajectory_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                           'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Define the target trajectory
        self._target_pose = [0.3, 0.2, 0.4, 0.0, 3.14, 0.0]
        
    def send_trajectory(self):
        point = JointTrajectoryPoint()
        point.positions = self._target_pose
        point.time_from_start = rclpy.duration.Duration(seconds=1.0)
        
        self._trajectory_msg.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._trajectory_msg
        
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        if result.error_code == FollowJointTrajectory.Result.SUCCESS:
            self.get_logger().info('Move successful')
        else:
            self.get_logger().error('Failed to move: %s' % result.error_code)

def main(args=None):
    rclpy.init(args=args)
    
    controller = UR5eArmController()
    controller.send_trajectory()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()