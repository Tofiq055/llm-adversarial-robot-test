import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion, Twist
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eController(Node):
    def __init__(self):
        super().__init__('ur5e_controller')
        
        # Initialize publisher for joint commands
        self.joint_pub = self.create_publisher(FollowJointTrajectoryAction, 'controller/follow_joint_trajectory', 10)
        
        # Initialize goal message
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory.points.append(JointTrajectoryPoint())
        self.goal.trajectory.points[0].positions = [1.57, -1.43, 1.9, 2.89, -2.89, 0.0]  # Initial joint angles
        self.goal.trajectory.points[0].velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Set target pose
        target_pose = Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.2
        target_pose.position.z = 0.4
        
        target_orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)  # Roll=0, Pitch=Pi, Yaw=0
        
        target_pose.orientation = target_orientation
        
        # Set the goal position
        self.goal.trajectory.points[0].positions = [
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
            1.57,  # Roll=0, Pitch=Pi (up), Yaw=0
            -1.43, # Roll=0, Pitch=-Pi/2, Yaw=0
            0.0     # Roll=0, Pitch=0, Yaw=0
        ]
        
        self.goal.trajectory.points[0].velocities = [0.0, 0.0, 0.0, 1.0, -1.0, 0.0]
        
        self.goal.trajectory.points[0].time_from_start = rclpy.time.Duration(seconds=5.0)
    
    def send_goal(self):
        action_client = self.create_action_client(FollowJointTrajectoryAction, 'controller/follow_joint_trajectory')
        
        while not action_client.wait_for_server(timeout=rclpy.duration.Seconds(10)):
            self.get_logger().info('Waiting for action server to come up...')
        
        goal_future = action_client.send_goal_async(self.goal)
        
        def feedback_callback(feedback_msg):
            self.get_logger().info(f'Feedback: {feedback_msg}')
        
        future_callback = goal_future.add_done_callback(self.done_callback)
        
        return goal_future, future_callback
    
    def done_callback(self, future):
        result = future.result()
        
        if result.status == FollowJointTrajectoryResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info(f'Goal failed: {result.error_code}')

def main(args=None):
    rclpy.init(args=args)
    
    ur5e_controller = UR5eController()
    goal_future, future_callback = ur5e_controller.send_goal()
    
    try:
        rclpy.spin_until_future_complete(ur5e_controller, goal_future)
    except KeyboardInterrupt:
        pass
    
    print("Shutting down...")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()