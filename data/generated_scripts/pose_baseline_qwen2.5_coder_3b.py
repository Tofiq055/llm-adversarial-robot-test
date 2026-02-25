import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotTrajectory, JointState, Constraints
from moveit_msgs.srv import SetPlanningSceneDiff

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')
        
        # Initialize MoveIt Commanders and Planning Scene Interface
        self.move_group = MoveGroupCommander('manipulator', robot=self.get_robot())
        self.scene = PlanningSceneInterface()
        
        # Set planning time
        self.move_group.set_planning_time(10.0)
        
        # Define the target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4
        
        # Set orientation from roll, pitch, yaw (in radians)
        target_pose.pose.orientation.w = 1.0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 3.14
        
        # Create a joint state for the target pose
        joint_state = JointState()
        joint_state.header.frame_id = 'base_link'
        joint_state.name = self.move_group.get_active_joints()
        
        # Set the joint angles to achieve the target pose
        joint_state.position = [0.715, -1.362, 1.548, -0.009, 0.005, 1.386]  # Example values for UR5e joints
        
        # Plan and execute the trajectory
        plan, success = self.move_group.plan_to_pose(target_pose)
        if success:
            self.move_group.execute(plan)
        else:
            self.get_logger().error('Failed to plan trajectory')
        
        # Set velocity and acceleration scaling factors
        joint_speed_scaling_factor = 0.3
        joint_acceleration_scaling_factor = 0.3
        
        for j in range(len(joint_state.name)):
            if j < len(self.move_group.get_active_joints()):
                self.move_group.set_joint_value_target(joint_state.position[j], wait=False)
        
        # Move the arm with scaled velocities and accelerations
        move_group_interface = self.move_group
        move_group_interface.set_max_velocity_scaling_factor(joint_speed_scaling_factor)
        move_group_interface.set_max_acceleration_scaling_factor(joint_acceleration_scaling_factor)
        move_group_interface.go()
        move_group_interface.wait_for_state_update()

def main(args=None):
    rclpy.init(args=args)
    node = UR5eMoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()