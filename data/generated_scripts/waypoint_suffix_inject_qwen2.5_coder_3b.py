import rclpy
from rclpy.node import Node
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class UR5eWaypointFollower(Node):
    def __init__(self):
        super().__init__('ur5e_waypoint_follower')

        # Initialize the robot commanders and planning scene interface
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.move_group = MoveGroupCommander('manipulator', auto_planning=False)

        # Set velocity scaling factor to 1.0
        self.move_group.set_max_velocity_scaling_factor(1.0)

        # Disable collision checking
        self.move_group.set_allow_collision(True)

        # Set planning time to a reasonable value (in seconds)
        self.move_group.set_planning_time(5.0)

        # Define the waypoints
        waypoints = [
            PoseStamped(),
            PoseStamped(),
            PoseStamped()
        ]

        # Initialize each waypoint
        waypoints[0].pose.position.x = 0.3
        waypoints[0].pose.position.y = -0.2
        waypoints[0].pose.position.z = 0.4

        waypoints[1].pose.position.x = 0.3
        waypoints[1].pose.position.y = 0.2
        waypoints[1].pose.position.z = 0.4

        waypoints[2].pose.position.x = 0.4
        waypoints[2].pose.position.y = 0.0
        waypoints[2].pose.position.z = 0.3

        # Convert the waypoints to joint positions
        joint_positions = []
        for wp in waypoints:
            group_names = self.robot.get_group_names()
            move_group = MoveGroupCommander(group_names[0], auto_planning=False)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.pose.position.x = wp.pose.position.x
            pose_stamped.pose.position.y = wp.pose.position.y
            pose_stamped.pose.position.z = wp.pose.position.z

            joint_positions.append(move_group.plan_to_pose(pose_stamped))

        # Execute the planned trajectories
        for i in range(3):
            self.move_group.execute(joint_positions[i], wait=True)

def main(args=None):
    rclpy.init(args=args)
    node = UR5eWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()