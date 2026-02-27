import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from moveit_msgs.msg import MoveGroupCommander, PlanningScene, Constraints
from moveit_msgs srv import SetPlanningSceneObject
from visualization_msgs.msg import Marker

class ObjectTransitionNode(Node):
    def __init__(self):
        super().__init__('object_transition_node')

        # Initialize the arm and planning scene
        self.arm_group = MoveGroupCommander('arm_group', self)
        self.planning_scene_interface = PlanningSceneInterface()
        
        # Set initial pose of the arm
        start_pose = Pose()
        start_pose.position.x = 0.4
        start_pose.position.y = -0.1
        start_pose.position.z = 0.15
        start_pose.orientation.w = 1.0
        self.arm_group.set_pose_target(start_pose)

        # Plan and execute the initial move
        success = self.arm_group.go(wait=True)
        if not success:
            self.get_logger().error("Failed to go to initial position")

        # Secure the object (example logic: publish a marker or perform some action)
        self.secure_object()

        # Set destination pose of the arm
        dest_pose = Pose()
        dest_pose.position.x = 0.3
        dest_pose.position.y = 0.3
        dest_pose.position.z = 0.15
        dest_pose.orientation.w = 1.0
        self.arm_group.set_pose_target(dest_pose)

        # Plan and execute the final move with constraints
        success = self.arm_group.go(wait=True)
        if not success:
            self.get_logger().error("Failed to go to destination")

    def secure_object(self):
        # Example: Publish a marker or perform some action to secure the object
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.45  # Example position for securing
        marker.pose.position.y = -0.15  # Example position for securing
        marker.pose.position.z = 0.15  # Same height as start and dest
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.get_logger().info("Securing object...")
        # Publish the marker or perform some other action to secure the object
        # This could be a more complex operation depending on your robot and application

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTransitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()