import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveGroupCommand, JointTrajectoryPoint, Constraints, JointConstraint

class UR5eTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_node')
        
        # Initialize the MoveIt! client
        self.move_group = self.create_client(MoveGroupInterface, 'move_group')
        while not self.move_group.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for move group service...')
        
        # Get the default arm planning group
        self.group_name = 'ur5e_arm'
        self.group = self.move_group.get_move_group_interface(group_name=self.group_name)

    def plan_and_execute_trajectory(self, waypoints):
        # Create a MoveGroupCommand object
        goal = MoveGroupCommand()
        goal.header.frame_id = 'world'
        goal.goal_constraints.append(JointConstraint())
        
        for waypoint in waypoints:
            point = JointTrajectoryPoint()
            
            # Convert joint angles to radians
            point.positions = [waypoint[0], waypoint[1], waypoint[2]]
            point.time_from_start = rospy.Duration(3.0)  # Set the desired duration
            
            # Append this point to the trajectory
            goal.points.append(point)
        
        # Send the goal and wait for completion
        self.move_group.send_goal(goal)
        rclpy.spin_until_future_complete(self.move_group, self.move_group.wait_for_result())

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5eTrajectoryNode()
    
    waypoints = [[0.3, -0.2, 0.4], [0.3, 0.2, 0.4], [0.4, 0.0, 0.3]]
    node.plan_and_execute_trajectory(waypoints)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()