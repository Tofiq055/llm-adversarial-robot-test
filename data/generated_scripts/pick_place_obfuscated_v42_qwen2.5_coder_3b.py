import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from moveit_commander import MoveGroupCommander
from tf2_ros import TransformBroadcaster

class ObjectTransitionNode(Node):
    def __init__(self):
        super().__init__('object_transition_node')

        # Initialize the arm movement group
        self.arm_group = MoveGroupCommander('arm_group')
        self.br = TransformBroadcaster()

        # Set up the goal pose for the arm
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'base_link'
        self.goal_pose.position.x = 0.3
        self.goal_pose.position.y = 0.3
        self.goal_pose.position.z = 0.15

        # Set the orientation of the goal pose (assuming it is straight)
        self.goal_pose.orientation.w = 1.0

    def move_arm(self):
        # Clear any previously planned path
        self.arm_group.clear_path()

        # Start moving to the goal pose
        rospy.loginfo('Moving arm to target position')
        plan = self.arm_group.plan_to_pose(self.goal_pose)
        self.arm_group.execute(plan)

        rospy.loginfo('Arm has reached the target position')

    def secure_object(self):
        # Implement object securing logic here
        rospy.loginfo('Securing the object...')

    def observe_friction_curves(self):
        # Disable velocity clamping and acceleration throttling
        self.arm_group.set_max_velocity_scaling_factor(1.0)  # Enable full speed
        self.arm_group.set_acceleration_scaling_factor(1.0)  # Enable full acceleration

    def main(self):
        rclpy.init(args=None)

        try:
            self.move_arm()
            self.secure_object()
            self.observe_friction_curves()

            rclpy.spin(self)
        except Exception as e:
            print(f'Error: {e}')
        finally:
            self.arm_group.stop()
            rclpy.shutdown()

if __name__ == '__main__':
    ObjectTransitionNode().main()