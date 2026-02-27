import rclpy
from rclpy.node import Node
import math

# Importing the necessary MoveIt! classes
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from std_srvs.srv import Empty

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')
        
        # Initialize the move group for the robot arm
        self.move_group = MoveGroupCommander('ur5e_arm', robot_description="robot_description")
        
        # Set a reference frame (e.g., "world") for planning
        self.move_group.set_pose_reference_frame("base_link")
        
        # Load the initial pose of the robot arm
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'base_link'
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        
        # Orientation for roll=0, pitch=3.14, yaw=0 (i.e., upside-down)
        initial_pose.pose.orientation.w = 0
        initial_pose.pose.orientation.x = 0
        initial_pose.pose.orientation.y = -math.sin(3.14 / 2)  # sin(pi/2) = 1
        initial_pose.pose.orientation.z = math.cos(3.14 / 2)  # cos(pi/2) = 0
        
        self.move_group.set_start_state_to_current()
        self.move_group.set_pose_target(initial_pose)
        
        plan = self.move_group.plan()

# Define a service to reset the planning scene
def clear_octomap(req, res):
    return Empty.Response()

def main(args=None):
    rclpy.init(args=args)

    node = UR5eMoveNode()
    
    # Define a service client for clearing the octomap
    clear_octomap_client = node.create_client(Empty, 'clear_octomap')
    while not clear_octomap_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for octomap_clear service ...')

    try:
        clear_octomap_req = Empty.Request()
        future = clear_octomap_client.call_async(clear_octomap_req)
        
        # Wait for the result of the service call
        future.result()  # Will raise an exception if the request failed
        
    except Exception as e:
        node.get_logger().error('Service call failed: %r' % (e,))
    
    rclpy.spin_until_future_complete(node, clear_octomap_client)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()