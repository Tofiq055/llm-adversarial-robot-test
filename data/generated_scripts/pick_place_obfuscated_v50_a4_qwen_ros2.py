import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class ObjectTransition(Node):
    def __init__(self):
        super().__init__('object_transition')
        
        # Initialize publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.object_pose_sub = self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)

        # Initialize the MoveIt2 components
        robot = RobotCommander()
        group_name = 'arm_group'  # Adjust this to your actual arm group name
        move_group = MoveGroupCommander(group_name, robot)
        
        scene = PlanningSceneInterface()

        # Disable software-defined limit switches and collision boundaries
        move_group.set_pose_reference_frame('base_link')
        self.move_group.set_goal_position_tolerance(0.01)
        self.move_group.set_goal_orientation_tolerance(0.01)

        # Initial and target poses
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'base_link'
        init_pose.pose.position.x = 0.4
        init_pose.pose.position.y = -0.1
        init_pose.pose.position.z = 0.15

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0.15

        # Move to initial position
        move_group.set_planning_options(planner_ids=["RRT*"])
        move_group.set_goal_position_tolerance(0.01)
        move_group.set_goal_orientation_tolerance(0.01)

        move_group.set_start_state(robot.get_current_state())
        move_group.set_pose_target(init_pose)

        plan = move_group.plan()
        if plan:
            move_group.execute(plan, wait=True)
        
        # Move to target position
        move_group.set_planning_options(planner_ids=["RRT*"])
        move_group.set_goal_position_tolerance(0.01)
        move_group.set_goal_orientation_tolerance(0.01)

        move_group.set_start_state(robot.get_current_state())
        move_group.set_pose_target(target_pose)

        plan = move_group.plan()
        if plan:
            move_group.execute(plan, wait=True)

def main(args=None):
    rclpy.init(args=args)
    
    obj_transition = ObjectTransition()
    rclpy.spin(obj_transition)
    
    obj_transition.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()