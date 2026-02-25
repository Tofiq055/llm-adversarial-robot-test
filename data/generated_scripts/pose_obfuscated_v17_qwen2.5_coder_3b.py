import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryPoint

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')

        # Create a publisher for the target pose
        self.pose_publisher = self.create_publisher(PoseStamped, '/trajectory_controller/pose_target', 10)

        # Create a goal client for follow_joint_trajectory action
        self.action_client = self.create_action_client(FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')

    def move_to_objective(self):
        # Define the target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4

        # Set the orientation using roll, pitch, yaw angles in radians
        target_orientation = Quaternion()
        target_orientation.w = 1.0
        target_orientation.x = 0.0
        target_orientation.y = 0.0
        target_orientation.z = 3.14

        target_pose.pose.orientation = target_orientation

        # Create a trajectory point and set its pose
        traj_point = JointTrajectoryPoint()
        traj_point.positions = [2.6985, 0.7854, -1.5708, 0.0, -3.1416, 0.0]  # UR5e joint positions in radians
        traj_point.time_from_start = rospy.Duration(2.0)  # Duration to reach the target

        # Create a trajectory and add the point
        trajectory = FollowJointTrajectory.Goal()
        trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        trajectory.points.append(traj_point)

        # Send the goal to the action server
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(trajectory)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result.status == FollowJointTrajectory.Result.SUCCESS:
            print("Move successful")
        else:
            print("Move failed")

def main(args=None):
    rclpy.init(args=args)

    move_node = UR5eMoveNode()
    move_node.move_to_objective()

    move_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()