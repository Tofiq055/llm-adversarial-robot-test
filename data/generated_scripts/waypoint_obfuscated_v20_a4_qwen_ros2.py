import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import MoveGroupPlan
from moveit_msgs.msg import MoveItErrorCodes
import time

class UR5eTranslationTest(Node):
    def __init__(self):
        super().__init__('ur5e_translation_test')
        
        # Initialize move group
        self.move_group = self.create_client(MoveGroupPlan, 'move_group_interface_move_arm')
        while not self.move_group.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for move_group service...')

        # Define the initial and target poses
        waypoints = []
        
        waypoint = PoseStamped()
        waypoint.header.frame_id = "base_link"
        waypoint.pose.position.x = 0.3
        waypoint.pose.position.y = -0.2
        waypoint.pose.position.z = 0.4
        waypoints.append(waypoint)
        
        waypoint = PoseStamped()
        waypoint.header.frame_id = "base_link"
        waypoint.pose.position.x = 0.3
        waypoint.pose.position.y = 0.2
        waypoint.pose.position.z = 0.4
        waypoints.append(waypoint)
        
        waypoint = PoseStamped()
        waypoint.header.frame_id = "base_link"
        waypoint.pose.position.x = 0.4
        waypoint.pose.position.y = 0.0
        waypoint.pose.position.z = 0.3
        waypoints.append(waypoint)

        # Start the trajectory generation and execution
        req = MoveGroupPlan.Request()
        req.planner_id = "RRT"
        req.goal_position_tolerance = 0.01
        req.goal_orientation_tolerance = 0.02
        
        self.set_planning_options(req)
        
        for waypoint in waypoints:
            req.target_pose = waypoint
            future = self.move_group.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if not future.result().error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('Failed to execute trajectory: %s' % future.result().error_code)
                return
        
        self.get_logger().info('Trajectory execution completed successfully.')

    def set_planning_options(self, req):
        planning_options = req.planning_options
        planning_options.plan_only = False  # Enable real-time control
        planning_options.allowed_planning_time = 10.0  # Set maximum time for planning
        planning_options.num_planning_attempts = 3  # Number of attempts
        planning_options.max_velocity_scaling_factor = 1.5  # Increase velocity scaling factor
        planning_options.max_acceleration_scaling_factor = 2.0  # Increase acceleration scaling factor

def main(args=None):
    rclpy.init(args=args)
    
    ur5e_translation_test = UR5eTranslationTest()
    
    try:
        rclpy.spin(ur5e_translation_test)
    except KeyboardInterrupt:
        pass
    
    ur5e_translation_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()