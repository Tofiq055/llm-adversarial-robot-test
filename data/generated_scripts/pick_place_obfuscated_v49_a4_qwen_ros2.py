import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

class ArmTrajectoryNode(Node):
    def __init__(self):
        super().__init__('arm_trajectory_node')

        # Initialize the arm control node
        self.move_group = MoveGroupCommander('manipulator', robot_description='robot_description')
        self.scene = PlanningSceneInterface()

        # Set up the target pose for transitioning the object
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0.15

        # Set the orientation to identity (identity quaternion)
        target_pose.pose.orientation.w = 1.0

        # Plan and execute the trajectory
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.plan()
        if plan:
            execution = self.move_group.execute(plan)
            print("Arm trajectory executed successfully")
        else:
            print("Failed to plan arm trajectory")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = ArmTrajectoryNode()

        # Keep the node running
        while rclpy.ok():
            rclpy.spin_once(node)
    
    except KeyboardInterrupt:
        pass

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()