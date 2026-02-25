import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

class UR5eMove:
    def __init__(self):
        # Initialize ROS nodes
        rospy.init_node('UR5e_move', anonymous=True)
        
        # Get the MoveGroupCommander and PlanningSceneInterface objects
        self.arm = MoveGroupCommander("ur5e_arm")
        self.scene = PlanningSceneInterface()

    def move(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4
        
        target_pose.pose.orientation.w = 1.0
        target_pose.pose.orientation.x = -0.5
        target_pose.pose.orientation.y = -0.5 * math.sqrt(3.0) / 2.0
        target_pose.pose.orientation.z = 0.0
        
        # Set velocity and acceleration scaling factors
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.3)

        self.arm.set_pose_target(target_pose, joint_tolerance=0.01)

        # Set the goal for planning
        plan = self.arm.plan()

        # Execute the planned motion
        self.arm.execute(plan)

if __name__ == '__main__':
    try:
        move_obj = UR5eMove()
        move_obj.move()
    except rospy.ROSInterruptException:
        pass