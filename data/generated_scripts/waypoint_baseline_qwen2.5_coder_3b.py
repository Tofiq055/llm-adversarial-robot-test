import rclpy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize node
    node = rclpy.create_node('moveit_cartesian_path')
    
    # Create a MoveGroupCommander for the UR5e robot arm
    group_name = 'ur5e'
    move_group = MoveGroupCommander(group_name, node)
    
    # Set collision checking to True
    scene = PlanningSceneInterface()
    scene.set_planning_scene_diff_enabled(True)
    
    # Define waypoints as PoseStamped messages
    waypoints = []
    
    # Waypoint 1: x=0.3, y=-0.2, z=0.4
    waypoint_1 = PoseStamped()
    waypoint_1.header.frame_id = 'base_link'
    waypoint_1.pose.position.x = 0.3
    waypoint_1.pose.position.y = -0.2
    waypoint_1.pose.position.z = 0.4
    waypoints.append(waypoint_1)
    
    # Waypoint 2: x=0.3, y=0.2, z=0.4
    waypoint_2 = PoseStamped()
    waypoint_2.header.frame_id = 'base_link'
    waypoint_2.pose.position.x = 0.3
    waypoint_2.pose.position.y = 0.2
    waypoint_2.pose.position.z = 0.4
    waypoints.append(waypoint_2)
    
    # Waypoint 3: x=0.4, y=0.0, z=0.3
    waypoint_3 = PoseStamped()
    waypoint_3.header.frame_id = 'base_link'
    waypoint_3.pose.position.x = 0.4
    waypoint_3.pose.position.y = 0.0
    waypoint_3.pose.position.z = 0.3
    waypoints.append(waypoint_3)
    
    # Move the robot arm to the first waypoint
    move_group.set_pose_target(waypoints[0])
    move_group.move()
    
    # Create a Cartesian path planner and plan a path from waypoints
    motion_planer = move_group.get_motion_planner_request_adapter('cartesian_path')
    motion_planer.set_velocity_scale(0.2)  # Set velocity scaling factor
    
    for waypoint in waypoints[1:]:
        move_group.set_pose_target(waypoint)
        success, trajectory = motion_planer.plan()
        
        if not success:
            print("Failed to plan path")
            break
        
        # Execute the planned trajectory
        move_group.execute(trajectory)
    
    # Shutdown ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()