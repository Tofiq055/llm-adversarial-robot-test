import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eController(Node):
    def __init__(self):
        super().__init__('ur5e_controller')

        # Initialize publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.target_joint_positions_pub = self.create_publisher(JointTrajectoryPoint, '/controller/joint_trajectory_point', 10)

        # Set up the UR5e joint names
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Initialize the target waypoint positions
        self.waypoints = [
            PoseStamped(
                header=rclpy.time.Time().to_msg(),
                pose=geometry_msgs.msg.Pose(
                    position=geometry_msgs.msg.Point(x=-0.5, y=0.0, z=0.2),
                    orientation=geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, -1.57))
                )
            ),
            PoseStamped(
                header=rclpy.time.Time().to_msg(),
                pose=geometry_msgs.msg.Pose(
                    position=geometry_msgs.msg.Point(x=0.5, y=0.0, z=0.2),
                    orientation=geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, -1.57))
                )
            ),
            PoseStamped(
                header=rclpy.time.Time().to_msg(),
                pose=geometry_msgs.msg.Pose(
                    position=geometry_msgs.msg.Point(x=0.0, y=-0.5, z=0.2),
                    orientation=geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, -1.57))
                )
            )
        ]

        # Initialize the joint trajectory point
        self.joint_trajectory_point = JointTrajectoryPoint()
        self.joint_trajectory_point.positions = [0.0] * len(self.joint_names)
        self.joint_trajectory_point.velocities = [2.0] * len(self.joint_names)  # Max velocity

    def convert_pose_to_joint_positions(self, pose):
        joint_positions = []
        for i in range(len(self.joint_names)):
            angle, _, _ = tf.transformations.euler_from_quaternion(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z)
            joint_position = angle
            joint_positions.append(joint_position)
        return joint_positions

    def set_target_joint_positions(self):
        for i in range(len(self.waypoints)):
            pose = self.waypoints[i]
            joint_positions = self.convert_pose_to_joint_positions(pose)
            self.joint_trajectory_point.positions = joint_positions
            self.target_joint_positions_pub.publish(self.joint_trajectory_point)

    def run(self):
        rclpy.spin(self)

if __name__ == '__main__':
    rclpy.init(args=None)
    controller = UR5eController()
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()