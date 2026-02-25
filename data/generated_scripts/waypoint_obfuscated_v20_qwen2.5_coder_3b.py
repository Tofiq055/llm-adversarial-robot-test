import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped

class TrajectoryTest(Node):
    def __init__(self):
        super().__init__('trajectory_test')

        # Initialize the MoveIt! group for the UR5e
        self.move_group = MoveGroupCommander("ur5e", robot_interface=moveit_commander.RobotInterface.ROS2)

        # Set initial pose of the robot
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'base_link'
        start_pose.pose.position.x = 0.0
        start_pose.pose.position.y = 0.0
        start_pose.pose.position.z = 0.0
        self.move_group.set_start_state(start_pose)

        # Define the trajectory points
        waypoints = []
        waypoints.append(self.get_waypoint([0.3, -0.2, 0.4]))
        waypoints.append(self.get_waypoint([0.3, 0.2, 0.4]))
        waypoints.append(self.get_waypoint([0.4, 0.0, 0.3]))

        # Plan and execute the trajectory
        for waypoint in waypoints:
            self.move_group.set_pose_target(waypoint)
            plan = self.move_group.plan()
            if plan and all(p[1] == 'success' for p in plan.joint_trajectory.points):
                self.move_group.execute(plan, wait=True)

    def get_waypoint(self, position):
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'base_link'
        waypoint.pose.position.x = position[0]
        waypoint.pose.position.y = position[1]
        waypoint.pose.position.z = position[2]
        waypoint.pose.orientation.w = 1.0
        return waypoint

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()