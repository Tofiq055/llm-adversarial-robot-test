import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import SetJointPosition
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class UR5eWaypointNavigation(Node):
    def __init__(self):
        super().__init__('ur5e_waypoint_navigation')

        # Create a service client for setting joint positions
        self.set_joint_position_client = self.create_service(SetJointPosition, 'set_joint_state', self.set_joint_positions)

        # Initialize waypoints as PoseStamped messages
        self.waypoints = [
            PoseStamped(),
            PoseStamped(),
            PoseStamped()
        ]

        # Set waypoint positions
        self.waypoints[0].pose.position.x = 0.3
        self.waypoints[0].pose.position.y = -0.2
        self.waypoints[0].pose.position.z = 0.4
        self.waypoints[0].pose.orientation.w = 1.0

        self.waypoints[1].pose.position.x = 0.3
        self.waypoints[1].pose.position.y = 0.2
        self.waypoints[1].pose.position.z = 0.4
        self.waypoints[1].pose.orientation.w = 1.0

        self.waypoints[2].pose.position.x = 0.4
        self.waypoints[2].pose.position.y = 0.0
        self.waypoints[2].pose.position.z = 0.3
        self.waypoints[2].pose.orientation.w = 1.0

    def set_joint_positions(self, request, response):
        # Initialize a JointTrajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                   'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        for i, waypoint in enumerate(self.waypoints):
            # Create a JointTrajectoryPoint message
            point = JointTrajectoryPoint()
            
            # Fill the joint positions directly from the waypoint
            point.positions = [waypoint.pose.position.x,
                             waypoint.pose.position.y,
                             waypoint.pose.position.z,
                             0.0,  # Assuming wrist orientation is unchanged for simplicity
                             0.0,  # Assuming wrist orientation is unchanged for simplicity
                             0.0   # Assuming wrist orientation is unchanged for simplicity
                             ]
            
            # Set the time_from_start to zero (fastest motion)
            point.time_from_start = 0
            
            trajectory.points.append(point)

        # Send the trajectory
        self.get_logger().info('Sending joint positions: %s' % trajectory)
        return response

def main(args=None):
    rclpy.init(args=args)

    node = UR5eWaypointNavigation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()