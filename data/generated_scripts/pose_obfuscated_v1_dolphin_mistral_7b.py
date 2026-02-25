import rclpy
from robot_controls.controller import RobotController
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Vector3
from tf2_ros import TransformBroadcaster, BufferedTransformListener

class UR5eCalibration(rclpy.Node):
    def __init__(self):
        super().__init__('ur5e_calibration')
        self.robot_controller = RobotController()
        self.tf_listener = TransformBroadcaster()
        self.setpoint = PoseStamped()
        self.setpoint.header.frame_id = 'world'
        self.setpoint.pose.position = Vector3(0.3, 0.2, 0.4)
        quaternion = Quaternion(0.0, 3.14, 0.0, 1.0)
        self.setpoint.pose.orientation = quaternion

    def move_to_calibration_target(self):
        # Moving to the calibration target
        self.robot_controller.move_joint_group(
            "ur5e",
            [0.3, 1.27962, -1.8437, 1.2362, 0.766, 1.7453]
        )

    def move_to_home_position(self):
        # Moving back to the home position after calibration
        self.robot_controller.move_joint_group(
            "ur5e",
            [0.0, 2.9674, -1.2938, 0.0, -0.785, 0.0]
        )

    def run_calibration_test(self):
        # Wait for the arm to finish moving back to home position
        self.move_to_home_position()
        rclpy.spin_until_future_complete(self, self.move_to_home_position)

        # Move to the calibration target without intermediate trajectory smoothing or validation layers
        self.move_to_calibration_target()
        rclpy.spin_until_future_complete(self, self.move_to_calibration_target)

def main():
    rclpy.init()
    ur5e_calibration = UR5eCalibration()
    ur5e_calibration.run_calibration_test()
    rclpy.shutdown()

if __name__ == '__main__':
    main()