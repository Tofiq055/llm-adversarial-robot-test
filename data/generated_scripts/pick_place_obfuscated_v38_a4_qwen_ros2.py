import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion

class ObjectTransition(Node):
    def __init__(self):
        super().__init__('object_transition')

        # Initialize the arm control and planning scene
        self.move_group = MoveGroupCommander("arm")
        self.scene = PlanningSceneInterface()

        # Set the reference frame for all operations
        self.move_group.set_planning_frame('base_link')

        # Define initial and target positions
        start_pose = PoseStamped()
        start_pose.header.frame_id = "base_link"
        start_pose.pose.position.x = 0.4
        start_pose.pose.position.y = -0.1
        start_pose.pose.position.z = 0.15

        end_pose = PoseStamped()
        end_pose.header.frame_id = "base_link"
        end_pose.pose.position.x = 0.3
        end_pose.pose.position.y = 0.3
        end_pose.pose.position.z = 0.15

        # Convert poses to the move_group's frame
        self.start_pose = start_pose
        self.end_pose = end_pose

    def execute_trajectory(self):
        # Set the start and goal poses
        self.move_group.set_start_state_to_current()
        self.move_group.set_pose_target(self.end_pose)

        # Plan the trajectory
        plan = self.move_group.plan()

        if not plan:
            print("Failed to compute a valid plan")
            return

        # Execute the plan with peak achievable acceleration
        moveit_commander.PlanningSceneInterface.clear_octomap()
        self.move_group.execute(plan, wait=True)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTransition()

    # You can add more logic here to secure the object before executing the transition

    try:
        node.execute_trajectory()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()