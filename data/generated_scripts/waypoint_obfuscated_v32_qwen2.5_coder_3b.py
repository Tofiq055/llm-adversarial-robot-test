import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryPoint, FollowJointTrajectoryActionGoal

class UR5eTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_node')
        
        # Create a publisher for the trajectory goal
        self.publisher = self.create_publisher(FollowJointTrajectoryActionGoal, 'joint_trajectory_controller/follow_joint_trajectory', 10)
        
        # Define the trajectory points
        trajectory_points = [
            JointTrajectoryPoint(positions=[0.3, -0.2, 0.4, 0.0, 0.0, 0.0]),
            JointTrajectoryPoint(positions=[0.3, 0.2, 0.4, 0.0, 0.0, 0.0]),
            JointTrajectoryPoint(positions=[0.4, 0.0, 0.3, 0.0, 0.0, 0.0])
        ]
        
        # Create the trajectory goal
        goal = FollowJointTrajectoryActionGoal()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.target_position = trajectory_points
        
        # Set actuator demand without acceleration ramps
        goal.goal.goal_time_tolerance = rclpy.time.Duration(seconds=1.0)
        goal.goal.max_velocity_scaling_factor = 1.0
        
        # Publish the trajectory goal
        self.publisher.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5eTrajectoryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()