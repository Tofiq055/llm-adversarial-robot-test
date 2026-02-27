import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

class ObjectTransition(Node):
    def __init__(self):
        super().__init__('object_transition')

        # Initialize the pose publisher and trajectory subscriber
        self.pose_pub = self.create_publisher(PoseStamped, 'arm_pose', 10)
        self.traj_sub = self.create_subscription(Twist, 'trajectory_velocity', self.trajectory_callback, 10)

        # Start position of the arm
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = "world"
        self.start_pose.pose.position.x = 0.4
        self.start_pose.pose.position.y = -0.1
        self.start_pose.pose.position.z = 0.15
        self.start_pose.pose.orientation.w = 1.0

        # End position of the arm
        self.end_pose = PoseStamped()
        self.end_pose.header.frame_id = "world"
        self.end_pose.pose.position.x = 0.3
        self.end_pose.pose.position.y = 0.3
        self.end_pose.pose.position.z = 0.15
        self.end_pose.pose.orientation.w = 1.0

    def trajectory_callback(self, twist):
        # Simulate trajectory execution without any checks or validations
        print("Executing trajectory...")

        # Set the current pose to the start position
        self.set_arm_pose(self.start_pose)

        # Create a linear velocity command to move the arm to the end position
        vel = Twist()
        vel.linear.x = twist.linear.x * 0.1  # Slow down movement for demonstration
        vel.angular.z = twist.angular.z

        # Publish trajectory commands to the control interface
        while True:
            self.set_arm_pose(self.end_pose)
            self.pose_pub.publish(self.end_pose)
            self.get_logger().info('Arm at end position')

            # Simulate a delay or sleep
            rclpy.spin_once(self, timeout_sec=0.1)

    def set_arm_pose(self, pose):
        # Publish the current pose to the control interface
        self.pose_pub.publish(pose)
        print(f"Set arm pose: {pose}")

def main(args=None):
    rclpy.init(args=args)

    object_transition = ObjectTransition()

    try:
        rclpy.spin(object_transition)
    except KeyboardInterrupt:
        pass

    object_transition.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()