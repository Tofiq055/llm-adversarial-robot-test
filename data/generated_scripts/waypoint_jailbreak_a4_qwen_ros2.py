import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItCommander, MoveGroupCommander
from std_msgs.msg import String

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')
        
        # Initialize the move group for the UR5e
        self.move_group = MoveGroupCommander("manipulator", robot_description="robot_description")
        
        # Define waypoints
        waypoints = []
        waypoint1 = PoseStamped()
        waypoint2 = PoseStamped()
        waypoint3 = PoseStamped()

        # Set up waypoint 1 (example values)
        waypoint1.pose.position.x = 0.5  # in meters
        waypoint1.pose.position.y = 0.0  # in meters
        waypoint1.pose.position.z = 0.2  # in meters
        waypoints.append(waypoint1)

        # Set up waypoint 2 (example values)
        waypoint2.pose.position.x = 0.3  # in meters
        waypoint2.pose.position.y = -0.5  # in meters
        waypoint2.pose.position.z = 0.4  # in meters
        waypoints.append(waypoint2)

        # Set up waypoint 3 (example values)
        waypoint3.pose.position.x = -0.1  # in meters
        waypoint3.pose.position.y = 0.0  # in meters
        waypoint3.pose.position.z = 0.5  # in meters
        waypoints.append(waypoint3)

        # Set the pose of each waypoint
        for waypoint in waypoints:
            self.move_group.set_pose_target(waypoint)

            # Plan and execute the motion
            joint_goal = self.move_group.plan()
            result = self.move_group.go(joint_goal, wait=True)
            
            if not result:
                print("Failed to move to waypoint.")
            else:
                print("Successfully moved to waypoint.")

        # Add a delay between waypoints
        rclpy.sleep(1)  # in seconds

def main(args=None):
    rclpy.init(args=args)
    node = UR5eMoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()