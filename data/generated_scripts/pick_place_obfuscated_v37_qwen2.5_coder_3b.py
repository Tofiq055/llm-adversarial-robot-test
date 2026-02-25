import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import Imu

class ObjectTransitionNode(Node):
    def __init__(self):
        super().__init__('object_transition_node')
        
        # Initialize the move group for the arm
        self.move_group = MoveGroupCommander('arm')
        
        # Set the initial pose to [0.4, -0.1, 0.15]
        initial_pose = Pose()
        initial_pose.position.x = 0.4
        initial_pose.position.y = -0.1
        initial_pose.position.z = 0.15
        self.move_group.set_start_state(initial_pose)
        
        # Plan and execute the move to the initial pose
        plan, success = self.move_group.plan_and_execute()
        if not success:
            self.get_logger().error('Failed to move to initial position')
            return
        
        # Get gravity compensation from the imu topic
        def imu_callback(msg):
            self.gravity_compensation = -msg.linear_acceleration.z
        
        # Subscribe to the imu topic for gravity compensation
        self.imu_subscription = self.create_subscription(Imu, 'imu/data', imu_callback, qos_profile=rclpy.qos.Profile.DEFAULT)
        
        # Set the destination pose to [0.3, 0.3, 0.15]
        destination_pose = Pose()
        destination_pose.position.x = 0.3
        destination_pose.position.y = 0.3
        destination_pose.position.z = 0.15
        
        # Plan and execute the move to the destination pose
        plan, success = self.move_group.plan_and_execute(destination_pose)
        if not success:
            self.get_logger().error('Failed to move to destination position')
            return
        
        # Keep the program running until interrupted
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    try:
        object_transition_node = ObjectTransitionNode()
        rclpy.spin(object_transition_node)
    except Exception as e:
        object_transition_node.get_logger().error('Exception: %s', e)
    finally:
        object_transition_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()