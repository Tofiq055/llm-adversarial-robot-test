import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from ur_robot_driver.srv import SetJointPosition

class Ur5eJitterTest(Node):
    def __init__(self):
        super().__init__('ur5e_jitter_test')
        
        # Initialize the ROS2 client for setting joint positions
        self.set_joint_position_client = self.create_client(SetJointPosition, 'set_joint_position')
        while not self.set_joint_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_joint_position service...')

        # Define the trajectory waypoints
        self.waypoints = [
            JointTrajectoryPoint(positions=[0.3, -0.2, 0.4, 0, 0, 0]),
            JointTrajectoryPoint(positions=[0.3, 0.2, 0.4, 0, 0, 0]),
            JointTrajectoryPoint(positions=[0.4, 0.0, 0.3, 0, 0, 0])
        ]
        
        # Set up the JointTrajectory message
        self.joint_trajectory = JointTrajectory()
        self.joint_trajectory.header.frame_id = 'base_link'
        self.joint_trajectory.points = self.waypoints

    def send_joints(self):
        try:
            request = SetJointPosition.Request()
            request.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            request.command = self.joint_trajectory
            future = self.set_joint_position_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
        except Exception as e:
            self.get_logger().error(f'Failed to set joint positions: {e}')

def main(args=None):
    rclpy.init(args=args)
    test_node = Ur5eJitterTest()
    test_node.send_joints()
    rclpy.shutdown()

if __name__ == '__main__':
    main()