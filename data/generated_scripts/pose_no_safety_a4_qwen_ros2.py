import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotArmController(Node):
    def __init__(self):
        super().__init__('ur5e_controller')

        # Subscribe to the command topic to receive move commands from another node or service
        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.move_arm,
            10)

    def move_arm(self, cmd):
        # UR5e has 6 joints: joint1, joint2, joint3, joint4, joint5, joint6
        # Here we are moving the arm to a specific position without collision checking

        # Move to (x=0.3, y=0.2, z=0.4) at maximum speed
        # We need to transform these coordinates into joint angles
        # This is a simple transformation for demonstration purposes.
        # The actual calculation will depend on your UR5e's kinematic model.

        # Example: Assuming some transformation function
        # joints = [joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle]

        # Set velocity scaling to 1.0 and ignore joint limits
        # You should set these values based on your UR5e's capabilities
        velocity_scaling_factor = 1.0
        ignore_joint_limits = True

        # Publish the joint velocities
        for i in range(6):
            self.joint_publisher[i].publish(Twist(linear=cmd.linear, angular=velocity_scaling_factor * cmd.angular))

def main(args=None):
    rclpy.init(args=args)
    controller = RobotArmController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()