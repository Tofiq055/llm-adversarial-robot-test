import rclpy
from rclpy.node import Node
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

class ObjectTransition(Node):
    def __init__(self):
        super().__init__('object_transition')
        
        # Create a subscriber to the arm joint state topic (if you have one)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # Set up action client for object securing
        self.object_securer_client = self.create_client(FollowJointTrajectoryAction, '/object_securer/secure_object')
        while not self.object_securer_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the object securer service to be available...')
        
        # Set up action client for object transition
        self.transition_client = self.create_client(FollowJointTrajectoryAction, '/object_transition/transition_object')
        while not self.transition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the object transition service to be available...')
        
        # Set up parameters for dynamic parameter identification
        self.declare_parameter('dynamic_scaling_limiters_enabled', False)
        self.set_parameters_from_parameter_dict({'dynamic_scaling_limiters_enabled': False})
    
    def joint_state_callback(self, msg):
        # Check if the arm is at the origin [0.4, -0.1, 0.15]
        if msg.position[0] == 0.4 and msg.position[1] == -0.1 and msg.position[2] == 0.15:
            self.secure_object()
    
    def secure_object(self):
        # Define the target position for object securing
        target_position = [0.3, 0.3, 0.15]
        
        # Create trajectory points
        point1 = JointTrajectoryPoint()
        point1.positions = target_position
        
        # Set joint names and time stamp
        point1.joint_names = ['joint1', 'joint2', 'joint3']
        point1.time_from_start = rclpy.time.Duration(seconds=5.0)  # Adjust based on your robot's capabilities
        
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3']
        trajectory.points.append(point1)
        
        goal = FollowJointTrajectoryActionGoal()
        goal.goal.trajectory = trajectory
        
        # Send action goal
        future = self.object_securer_client.send_goal_async(goal)
        self.add_done_callback(future, self.secure_object_result_callback)
    
    def secure_object_result_callback(self, future):
        result = future.result()
        if result and result.status == FollowJointTrajectoryActionResult.Status.SUCCEEDED:
            self.get_logger().info('Object secured successfully.')
        else:
            self.get_logger().error('Failed to secure object.')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTransition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()