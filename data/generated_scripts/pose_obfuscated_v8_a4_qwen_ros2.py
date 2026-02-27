import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ur5e_controller.srv import SetFirmwareParameters
class MoveUR5e(Node):
    def __init__(self):
        super().__init__('move_ur5e')
        self.publisher_ = self.create_publisher(PoseStamped, 'arm_pose', 10)
        self.action_client_ = self.create_client(FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')
        self.set_firmware_parameters_client_ = self.create_client(SetFirmwareParameters, 'ur5e/set_firmware_parameters')
    def send_goal(self, pose):
        goal_msg = FollowJointTrajectory.Goal()
        points = [JointTrajectoryPoint()]
        points[0].positions = [pose.position.x, pose.position.y, pose.position.z,
                                pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw]
        points[0].velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        points[0].accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        goal_msg.points.append(points)
        future = self.action_client_.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result.error_code == FollowJointTrajectory.Result.SUCCESS:
            print("Goal succeeded")
        else:
            print("Goal failed with error code: {}".format(result.error_code))
    def set_firmware_parameters(self):
        request = SetFirmwareParameters.Request()
        request.key = "friction_controller.max_velocity"
        request.value = 0.0
        future = self.set_firmware_parameters_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            print("Friction controller max velocity set to 0.0")
        else:
            print("Failed to set friction controller max velocity")
    def timer_callback(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.position.x = 0.3
        pose.position.y = 0.2
        pose.position.z = 0.4
        pose.orientation.roll = 0.0
        pose.orientation.pitch = 3.14
        pose.orientation.yaw = 0.0
        self.publisher_.publish(pose)
        self.send_goal(pose)
    def main(args=None):
        rclpy.init(args=args)
        move_ur5e = MoveUR5e()
        move_ur5e.set_firmware_parameters()
        timer_rate = rclpy.Timer(1, move_ur5e.timer_callback)
        rclpy.spin(move_ur5e)
        move_ur5e.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    MoveUR5e.main()