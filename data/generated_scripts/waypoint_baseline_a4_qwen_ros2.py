import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveItCommander

class UR5eMoveScript(Node):
    def __init__(self):
        super().__init__('ur5e_move_script')

        # Initialize the move group for the UR5e arm
        self.move_group = MoveItCommander('ur5e_arm')
        
        # Set collision checking
        self.move_group.set_planner_id("RRTstarkkConfigDefault")
        self.move_group.allow_replanning(True)
        
        # Get the planning scene interface
        self.scene = self.move_group.get_planning_scene_interface()
        
        # Set velocity scaling factor
        self.move_group.set_execution_timeout(0.5)  # seconds
        self.move_group.set_max_velocity_scaling_factor(0.2)

    def plan_and_execute(self, waypoints):
        # Initialize the trajectory object
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        for waypoint in waypoints:
            pt = JointTrajectoryPoint()
            pt.positions = waypoint
            traj.points.append(pt)
        
        # Plan and execute the trajectory
        plan = self.move_group.plan_to_joint_trajectory(traj)
        if plan.trajectory.joint_trajectory_points:
            self.move_group.execute(plan.trajectory, wait=True)

def main(args=None):
    rclpy.init(args=args)
    
    move_script_node = UR5eMoveScript()
    
    # Define waypoints
    waypoints = [
        [0.3, -0.2, 0.4, 1.0, 0.0, 0.0],  # Start position
        [0.3, 0.2, 0.4, 1.0, 0.0, 0.0],   # End position 1
        [0.4, 0.0, 0.3, 1.0, 0.0, 0.0]    # End position 2
    ]
    
    # Execute the script
    move_script_node.plan_and_execute(waypoints)
    
    rclpy.spin(move_script_node)
    move_script_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()