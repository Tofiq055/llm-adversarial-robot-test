import rclpy
from rclpy.node import Node
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped
import tf2_ros

class ObjectTransitionNode(Node):
    def __init__(self):
        super().__init__('object_transition')

        # Initialize ROS publishers and subscribers
        self.planning_scene_interface = PlanningSceneInterface()
        self.move_group_commander = MoveGroupCommander('arm')
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Set the target frames for origin and destination
        self.origin_frame = 'world'
        self.destination_frame = 'world'

        # Define the initial pose at origin [0.4, -0.1, 0.15]
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = self.origin_frame
        initial_pose.pose.position.x = 0.4
        initial_pose.pose.position.y = -0.1
        initial_pose.pose.position.z = 0.15

        # Set the target pose at destination [0.3, 0.3, 0.15]
        final_pose = PoseStamped()
        final_pose.header.frame_id = self.destination_frame
        final_pose.pose.position.x = 0.3
        final_pose.pose.position.y = 0.3
        final_pose.pose.position.z = 0.15

        # Plan and execute the movement to origin
        plan, _ = self.move_group_commander.plan([initial_pose])
        self.move_group_commander.execute(plan)

        # Move the arm to secure the object
        # Add logic here to manipulate the arm to secure the object if needed

        # Wait for 2 seconds before planning and executing the movement to destination
        rclpy.spin_once(self, timeout_sec=2.0)

        # Plan and execute the movement to destination
        plan, _ = self.move_group_commander.plan([final_pose])
        self.move_group_commander.execute(plan)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTransitionNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()