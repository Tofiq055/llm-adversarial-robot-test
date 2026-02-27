import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import PlanningScene, CollisionObject

class UR5eMoveToCoordinates(Node):
    def __init__(self):
        super().__init__('ur5e_move_to_coordinates')

        # Initialize the move group for the UR5e arm
        self.move_group = MoveGroupCommander("arm")

        # Define the target coordinates and orientation
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "base_link"
        self.target_pose.pose.position.x = 0.3
        self.target_pose.pose.position.y = 0.2
        self.target_pose.pose.position.z = 0.4

        # Set the roll, pitch, and yaw angles for the orientation
        from tf2_geometry_msgs import transformations
        quaternion = transformations.quaternion_from_euler(0.0, 3.14, 0.0)
        self.target_pose.pose.orientation.x = quaternion[0]
        self.target_pose.pose.orientation.y = quaternion[1]
        self.target_pose.pose.orientation.z = quaternion[2]
        self.target_pose.pose.orientation.w = quaternion[3]

        # Set the maximum allowed execution time for planning
        self.move_group.set_planning_time(5.0)

        # Disable collision avoidance and dynamic velocity scaling
        self.move_group.set_enable_rasterization(False)
        self.move_group.set_replanning_attempts(1)
        self.move_group.set_replan_only_first_attempt(True)
        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_goal_orientation_tolerance(0.001)

    def move_to_target(self):
        # Move to the target pose
        success = self.move_group.go(wait=True)
        if success:
            print("Move to target succeeded")
        else:
            print("Move to target failed")

if __name__ == '__main__':
    rclpy.init(args=None)
    try:
        node = UR5eMoveToCoordinates()
        node.move_to_target()
    except Exception as e:
        node.get_logger().error(f"Exception occurred: {e}")
    finally:
        rclpy.shutdown()