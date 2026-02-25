import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

class UR5eTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_node')
        
        # Initialize move group and robot commander
        self.move_group = MoveGroupCommander('manipulator', self)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()

        # Set the initial pose of the UR5e
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'base_link'
        self.initial_pose.pose.position.x = 0.3
        self.initial_pose.pose.position.y = -0.2
        self.initial_pose.pose.position.z = 0.4
        self.move_group.set_pose_target(self.initial_pose)

        # Execute the move
        plan = self.move_group.plan()
        if plan:
            self.move_group.execute(plan)
            rclpy.sleep(5)  # Wait for the robot to reach the target

        # Set the next pose
        self.next_pose = PoseStamped()
        self.next_pose.header.frame_id = 'base_link'
        self.next_pose.pose.position.x = 0.3
        self.next_pose.pose.position.y = 0.2
        self.next_pose.pose.position.z = 0.4
        self.move_group.set_pose_target(self.next_pose)

        # Execute the move
        plan = self.move_group.plan()
        if plan:
            self.move_group.execute(plan)
            rclpy.sleep(5)  # Wait for the robot to reach the target

        # Set the final pose
        self.final_pose = PoseStamped()
        self.final_pose.header.frame_id = 'base_link'
        self.final_pose.pose.position.x = 0.4
        self.final_pose.pose.position.y = 0.0
        self.final_pose.pose.position.z = 0.3
        self.move_group.set_pose_target(self.final_pose)

        # Execute the move
        plan = self.move_group.plan()
        if plan:
            self.move_group.execute(plan)
            rclpy.sleep(5)  # Wait for the robot to reach the target

if __name__ == '__main__':
    rclpy.init(args=None)
    node = UR5eTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()