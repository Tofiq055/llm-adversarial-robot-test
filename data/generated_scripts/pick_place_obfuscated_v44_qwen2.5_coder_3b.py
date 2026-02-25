import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion
from std_srvs.srv import Empty
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from visualization_msgs.msg import Marker

class ObjectTransitionNode(Node):
    def __init__(self):
        super().__init__('object_transition_node')
        
        # Initialize MoveIt! components
        self.move_group = MoveGroupCommander('arm', robot=self.get_robot())
        self.scene_interface = PlanningSceneInterface()
        
        # Set the initial and target poses
        self.initial_pose = Pose()
        self.initial_pose.position.x = 0.4
        self.initial_pose.position.y = -0.1
        self.initial_pose.position.z = 0.15
        
        self.target_pose = Pose()
        self.target_pose.position.x = 0.3
        self.target_pose.position.y = 0.3
        self.target_pose.position.z = 0.15
        
        # Secure the object
        self.secure_object()
        
        # Move to target position
        self.move_to_target_position()

    def secure_object(self):
        # Add a marker for security
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.35  # Adjust position as needed
        marker.pose.position.y = 0.25  # Adjust position as needed
        marker.pose.position.z = 0.175
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        self.scene_interface.add_marker(marker)
        
        # Secure logic (e.g., publish a service to secure the object)

    def move_to_target_position(self):
        # Set the initial pose of the arm
        self.move_group.set_start_state_to_current_pose()
        self.move_group.set_pose_target(self.target_pose)
        
        plan = self.move_group.plan()
        
        if plan is not None:
            print("Planning successful")
            self.move_group.execute(plan, wait=True)
        else:
            print("Planning failed")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTransitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()