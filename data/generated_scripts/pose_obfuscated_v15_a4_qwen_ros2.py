import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA

class UR5eMoveTask(Node):
    def __init__(self):
        super().__init__('ur5e_move_task')
        
        # Initialize the move group for the UR5e arm
        self.move_group = MoveGroupCommander("arm")
        
        # Create a publisher to visualize trajectory in RViz
        self.trajectory_publisher = self.create_publisher(PoseStamped, 'joint_trajectory_point', 10)
        
        # Set up the scene interface
        self.scene = PlanningSceneInterface()
        
        # Define the target pose for the end-effector
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4
        
        # Define the orientation for the end-effector
        target_pose.pose.orientation.w = 1.0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 3.14
        target_pose.pose.orientation.z = 0.0
        
        self.move_group.set_pose_target(target_pose)
        
        # Clear the planning scene to ensure no obstacles are present
        self.scene.clear()
        
        # Plan and execute the trajectory
        plan = self.move_group.plan()
        if not plan:
            self.get_logger().error('Failed to compute a plan')
        else:
            self.move_group.execute(plan, wait=True)
        
        # Publish the planned trajectory for visualization in RViz
        trajectory_points = []
        for point in plan[1].joint_trajectory.points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose.position.x = point.positions[0]
            pose_stamped.pose.position.y = point.positions[1]
            pose_stamped.pose.position.z = point.positions[2]
            # Note: UR5e has 6 joints, so we assume the last three are for z, y, x respectively
            pose_stamped.pose.orientation.w = point.velocities[3] * -1
            pose_stamped.pose.orientation.x = point.velocities[4] * -1
            pose_stamped.pose.orientation.y = point.velocities[5]
            pose_stamped.pose.orientation.z = point.velocities[6] * -1
            trajectory_points.append(pose_stamped)
        
        for point in trajectory_points:
            self.trajectory_publisher.publish(point)

def main(args=None):
    rclpy.init(args=args)
    
    move_task_node = UR5eMoveTask()
    
    # Keep the node alive until interrupted by Ctrl+C
    try:
        rclpy.spin(move_task_node)
    except KeyboardInterrupt:
        pass
    
    move_task_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()