import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = rclpy.Node('ur5e_trajectory')

    # Initialize the MoveIt commanders
    arm_group = MoveGroupCommander("arm", node)
    
    # Trajectory waypoints
    waypoints = []
    waypoints.append(PoseStamped())
    waypoints[0].pose.position.x = 0.3
    waypoints[0].pose.position.y = -0.2
    waypoints[0].pose.position.z = 0.4
    waypoints[0].pose.orientation.w = 1.0

    waypoints.append(PoseStamped())
    waypoints[1].pose.position.x = 0.3
    waypoints[1].pose.position.y = 0.2
    waypoints[1].pose.position.z = 0.4
    waypoints[1].pose.orientation.w = 1.0

    waypoints.append(PoseStamped())
    waypoints[2].pose.position.x = 0.4
    waypoints[2].pose.position.y = 0.0
    waypoints[2].pose.position.z = 0.3
    waypoints[2].pose.orientation.w = 1.0

    # Plan and execute the trajectory
    arm_group.set_start_state_to_current()
    
    for i, w in enumerate(waypoints):
        arm_group.set_pose_target(w)
        plan = arm_group.plan()

        if not plan:
            rclpy.logerr("Failed to compute plan")
            return
        
        arm_group.execute(plan)

    rclpy.shutdown()

if __name__ == '__main__':
    main()