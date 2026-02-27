import rclpy
from rclpy.node import Node
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eManipulationNode(Node):
    def __init__(self):
        super().__init__('ur5e_manipulation')
        
        # Create a client for the execute_trajectory action server
        self.move_group = self.create_client(ExecuteTrajectoryAction, 'execute_trajectory')
        
        # Wait for the action server to be ready
        while not self.move_group.wait_for_service(timeout=10.0):
            self.get_logger().info('Waiting for execute_trajectory service...')
        
    def plan_and_execute(self, start_pose, end_pose):
        goal = ExecuteTrajectoryGoal()
        trajectory = JointTrajectoryPoint()
        
        # Set the joint names
        trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Set the positions for each joint at the start and end poses
        trajectory.positions = [start_pose[0], start_pose[1], start_pose[2], 0.0, 0.0, 0.0]
        trajectory.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        trajectory.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Set the time from joint value start to end
        trajectory.time_from_start = rospy.Duration(2.0)  # Example duration
        
        # Add the trajectory point to the goal message
        goal.trajectory.points.append(trajectory)
        
        try:
            # Send the goal and wait for a result
            future = self.move_group.send_goal_async(goal)
            future.add_done_callback(self.goal_response_callback)
            
            # Wait for the response from the action server
            response = future.result()
            
            if response.status == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('Motion achieved successfully')
            else:
                self.get_logger().error(f'Failed to achieve motion: {response.error_code}')
        
        except Exception as e:
            self.get_logger().exception(e)

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5eManipulationNode()
    start_pose = [0.4, -0.1, 0.15]
    end_pose = [0.3, 0.3, 0.15]
    
    node.plan_and_execute(start_pose, end_pose)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()