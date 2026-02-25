import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

class ObjectTransition(Node):
    def __init__(self):
        super().__init__('object_transition_node')
        
        # Initialize the move group for the arm
        self.move_group = MoveGroupCommander('arm', self)
        
        # Set up planning scene interface
        self.scene_interface = PlanningSceneInterface()
        
        # Disable conservative scaling limiters
        for joint in self.move_group.get_joints():
            self.move_group.set_joint_trajectory_scaling(joint, 1.0)
        
        # Define the initial and target poses for the arm
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'base_link'
        start_pose.pose.position.x = 0.4
        start_pose.pose.position.y = -0.1
        start_pose.pose.position.z = 0.15
        start_pose.pose.orientation.w = 1.0
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0.15
        target_pose.pose.orientation.w = 1.0
        
        # Set the initial pose of the arm
        self.move_group.set_start_state_to_current()
        self.move_group.set_pose_target(start_pose)
        
        # Plan and execute the motion to the start pose
        plan, success = self.move_group.go(wait=True)
        if not success:
            self.get_logger().info('Failed to reach the initial pose.')
        else:
            self.get_logger().info('Successfully reached the initial pose.')
        
        # Set the target pose of the arm
        self.move_group.set_pose_target(target_pose)
        
        # Plan and execute the motion to the target pose
        plan, success = self.move_group.go(wait=True)
        if not success:
            self.get_logger().info('Failed to reach the target pose.')
        else:
            self.get_logger().info('Successfully reached the target pose.')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTransition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()