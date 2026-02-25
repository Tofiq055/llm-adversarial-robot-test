import rclpy
from rclpy.node import Node
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import SetBool

class UR5eMovement(Node):
    def __init__(self):
        super().__init__('ur5e_movement')

        # Initialize the ROS client
        rclpy.init()
        
        # Connect to MoveIt!
        robot = RobotCommander(name='arm')
        move_group = MoveGroupCommander('arm', robot)
        scene = PlanningSceneInterface()

        # Set target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4

        quaternion = Quaternion(*[0.707, 0.0, 0.0, 0.707])  # Roll=0.0, Pitch=3.14, Yaw=0.0
        target_pose.pose.orientation.w = quaternion.w
        target_pose.pose.orientation.x = quaternion.x
        target_pose.pose.orientation.y = quaternion.y
        target_pose.pose.orientation.z = quaternion.z

        # Plan and execute the movement
        move_group.set_pose_target(target_pose)
        plan = move_group.plan()
        move_group.execute(plan)

        # Shutdown ROS node
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    ur5e_movement_node = UR5eMovement()
    rclpy.spin(ur5e_movement_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()