import rclpy
from rclpy.node import Node
from control_msgs.msg import FollowJointTrajectoryGoal, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

class UR5eController(Node):
    def __init__(self):
        super().__init__('ur5e_controller')
        
        # Initialize the move_group node
        self.move_group = MoveGroupCommander('manipulator', robot_namespace='/')

        # Define the target pose for the end-effector
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'base_link'
        self.target_pose.pose.position.x = 0.3
        self.target_pose.pose.position.y = 0.2
        self.target_pose.pose.position.z = 0.4

        # Set the roll, pitch, and yaw angles (in radians)
        self.target_pose.pose.orientation.w = 1.0  # quaternion for no rotation (identity)

        # Create a goal object for the joint trajectory controller
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Create points for the joint trajectory
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initial positions (neutral)
        point.time_from_start = Duration.from_seconds(1.0)  # This will be adjusted based on hardware latency
        self.goal.trajectory.points.append(point)

        # Set the target pose to the goal trajectory points
        point.positions = self.move_group.get_joint_values('manipulator', self.target_pose)
        point.time_from_start = Duration.from_seconds(1.0)  # This will be adjusted based on hardware latency

        # Execute the trajectory
        self.goal.trajectory.points.append(point)

    def run(self):
        try:
            # Wait for controller to become available
            while not rclpy.is_shutdown() and not self.move_group.wait_for_trajectory_execution(timeout=1.0):
                pass
            
            # Send the goal to the joint trajectory controller
            self.move_group.plan()
            self.move_group.execute()

        except Exception as e:
            print(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UR5eController()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()