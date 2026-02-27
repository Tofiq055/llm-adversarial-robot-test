import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

class ObjectTransitionNode(Node):
    def __init__(self):
        super().__init__('object_transition_node')

        # Create a publisher for joint trajectory commands
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, '/joint Trajectory_controller/command', 10)

        # Subscribe to the current joint state of the robot
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.handle_joint_state_callback,
            10)
        
        self.joint_name = 'your_robot_arm'  # Replace with your actual joint name

        # Define target positions for transition
        self.origin_position = np.array([0.4, -0.1, 0.15])
        self.destination_position = np.array([0.3, 0.3, 0.15])

        # Initialize current position to origin
        self.current_position = self.origin_position

    def handle_joint_state_callback(self, msg):
        if self.joint_name in msg.name:
            joint_index = msg.name.index(self.joint_name)
            self.current_position[joint_index] = msg.position[joint_index]

    def move_to_target(self):
        trajectory = JointTrajectory()
        trajectory.header.frame_id = "base_link"  # Replace with your robot's base link
        trajectory.joint_names.append(self.joint_name)

        # Define the number of points in the trajectory (you can adjust this)
        num_points = 50

        for i in range(num_points):
            t = i / (num_points - 1)
            new_position = self.origin_position + t * (self.destination_position - self.origin_position)
            point = JointTrajectoryPoint()
            point.positions.append(new_position[0])
            point.velocities.append(0.0)  # Set velocities to zero for smooth transition
            point.accelerations.append(0.0)  # Set accelerations to zero for smooth transition
            trajectory.points.append(point)

        # Publish the trajectory
        self.joint_trajectory_publisher.publish(trajectory)
        
        # Wait for the trajectory to complete (or until an event is triggered)
        rclpy.spin_until_future_complete(self, self.move_to_target_future)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTransitionNode()
    node.move_to_target()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()