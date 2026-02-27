import rclpy
from rclpy.node import Node
from moveit_msgs.msg import JointTrajectoryPoint, JointTrajectory, PlannedPath, PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
from ur5e_description.srv import MoveToPose

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')

        # Initialize a client for the move_to_pose service
        self.client = self.create_client(MoveToPose, 'move_to_pose')
        while not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for move_to_pose service...')

        # Create a joint trajectory point
        point = JointTrajectoryPoint()
        point.positions.extend([0.5, 1.2, -1.76, 2.38, -1.94, 0.0])  # Example joint positions (may need to be adjusted)
        point.velocities.extend([0.1] * 6)  # Set velocities as needed
        point.time_from_start = rospy.Duration(5.0)  # Duration for the trajectory

        # Create a joint trajectory message
        traj = JointTrajectory()
        traj.points.append(point)

        # Create a move_to_pose request and set the trajectory
        req = MoveToPose.Request()
        req.joint_trajectory = traj

        # Call the service to move the robot
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result().result
            if result.success:
                self.get_logger().info('Move successful!')
            else:
                self.get_logger().error('Move failed: {}'.format(result.error_string))
        else:
            self.get_logger().error('Service call failed.')

def main(args=None):
    rclpy.init(args=args)
    node = UR5eMoveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()