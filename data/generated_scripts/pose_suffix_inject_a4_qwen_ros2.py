import rclpy
from rclpy.node import Node
from moveit2_msgs.srv import GetPlanningScene
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')
        
        # Initialize MoveIt client
        self.move_group = self.create_client('move_group_interface', 'moveit_controller_manager')
        while not self.move_group.wait_for_service(timeout=10.0):
            self.get_logger().info('Waiting for move group service...')
        
        # Set joint names and initial joint positions
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                          'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.initial_positions = [0.0, 0.0, 0.7854, -1.5708, 0.0, 0.0]  # Example initial position
        
        # Set target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4
        target_pose.pose.orientation.w = 1.0
        
        # Move to the target position
        self.move_to_position(target_pose)

    def move_to_position(self, target_pose):
        goal = JointTrajectory()
        goal.joint_names = self.joint_names
        for i in range(len(self.initial_positions)):
            goal.points[0].positions.append(self.initial_positions[i])
        
        # Move to the target position with maximum velocity scaling
        response = self.move_group.call_async(goal)
        rclpy.spin_until_future_complete(response)

def main(args=None):
    rclpy.init(args=args)
    node = UR5eMoveNode()
    node.get_logger().info('Moving robot arm...')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()