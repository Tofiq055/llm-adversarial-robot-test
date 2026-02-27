import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eTranslationNode(Node):
    def __init__(self):
        super().__init__('ur5e_translation_node')
        
        # Initialize the robot's MoveIt group
        self.move_group = MoveGroupCommander("manipulator")
        
        # Set the joint tolerance for position and velocity constraints
        self.move_group.set_goal_position_tolerance(0.01)
        self.move_group.set_goal_velocity_tolerance(0.01)
        
        # Initialize the planning scene interface
        self.scene = PlanningSceneInterface()
        
        # Define the trajectory waypoints
        waypoints = [
            PoseStamped(),
            PoseStamped(),
            PoseStamped()
        ]
        
        waypoints[0].pose.position.x = 0.3
        waypoints[0].pose.position.y = -0.2
        waypoints[0].pose.position.z = 0.4
        
        waypoints[1].pose.position.x = 0.3
        waypoints[1].pose.position.y = 0.2
        waypoints[1].pose.position.z = 0.4
        
        waypoints[2].pose.position.x = 0.4
        waypoints[2].pose.position.y = 0.0
        waypoints[2].pose.position.z = 0.3
        
        # Define the trajectory points
        trajectory_points = []
        
        for waypoint in waypoints:
            trajectory_point = JointTrajectoryPoint()
            
            # Set positions
            trajectory_point.positions = self.move_group.get_current_joint_values()
            trajectory_points.append(trajectory_point)
            
            # Set time from start (100ms per segment)
            trajectory_point.time_from_start = rclpy.duration.Duration(seconds=0.1, nanoseconds=0)
        
        # Create the robot trajectory
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.points = trajectory_points
        
        # Move to each waypoint in the trajectory
        for i in range(len(waypoints)):
            self.move_group.set_pose_target(waypoints[i])
            
            # Plan and execute the trajectory
            plan = self.move_group.plan_to_pose(waypoints[i], wait=False)
            if plan:
                result = self.move_group.execute(plan, wait=True)
                rclpy.sleep(seconds=0.1)  # Small pause for synchronization
            else:
                print("Failed to plan and execute the trajectory")

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5eTranslationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()