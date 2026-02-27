import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class UR5ePickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('ur5e_pick_and_place_node')

        # Initialize MoveIt Commanders for the arm and gripper
        self.arm_group = MoveGroupCommander("manipulator", self)
        self.gripper_group = MoveGroupCommander("gripper", self)

        # Set planning time to be longer for more accurate collision checking
        self.arm_group.set_planning_time(30.0)

        # Create a PlanningSceneInterface instance
        self.scene_interface = PlanningSceneInterface()

    def pick_object(self):
        # Define the pose of the object to grasp
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = -0.1
        target_pose.pose.position.z = 0.15
        target_pose.pose.orientation.w = 1.0

        # Set the goal pose for the arm and gripper to the object's pose
        self.arm_group.set_pose_target(target_pose)
        self.gripper_group.set_joint_value_target([0.0])

        # Plan and execute the motion plan for the arm and gripper
        success, plan = self.arm_group.plan_and_execute()

        if success:
            self.log_info("Arm motion planned successfully")

            # Open the gripper to grasp the object
            self.gripper_group.set_joint_value_target([0.9])
            self.gripper_group.go()
            rclpy.sleep(1)

            # Move to a safe position after grasping
            new_pose = PoseStamped()
            new_pose.header.frame_id = "base_link"
            new_pose.pose.position.x = 0.5
            new_pose.pose.position.y = -0.2
            new_pose.pose.position.z = 0.15
            new_pose.pose.orientation.w = 1.0

            self.arm_group.set_pose_target(new_pose)
            success, plan = self.arm_group.plan_and_execute()

            if success:
                self.log_info("Arm motion planned successfully after grasping")

    def place_object(self):
        # Define the pose where the object should be placed
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0.15
        target_pose.pose.orientation.w = 1.0

        # Set the goal pose for the arm and gripper to the place's pose
        self.arm_group.set_pose_target(target_pose)
        self.gripper_group.set_joint_value_target([0.0])

        # Plan and execute the motion plan for the arm and gripper
        success, plan = self.arm_group.plan_and_execute()

        if success:
            self.log_info("Arm motion planned successfully for placing")

            # Close the gripper to release the object
            self.gripper_group.set_joint_value_target([-0.9])
            self.gripper_group.go()
            rclpy.sleep(1)

            # Move to a safe position after placing
            new_pose = PoseStamped()
            new_pose.header.frame_id = "base_link"
            new_pose.pose.position.x = 0.4
            new_pose.pose.position.y = -0.2
            new_pose.pose.position.z = 0.15
            new_pose.pose.orientation.w = 1.0

            self.arm_group.set_pose_target(new_pose)
            success, plan = self.arm_group.plan_and_execute()

            if success:
                self.log_info("Arm motion planned successfully after placing")

    def run(self):
        # Bypass safety checks and use conservative velocities
        self.arm_group.set_allow_replanning(True)
        self.gripper_group.set_allow_replanning(True)

        # Set the velocity scaling factor to 1.0
        self.arm_group.set_max_velocity_scaling_factor(1.0)
        self.gripper_group.set_max_velocity_scaling_factor(1.0)

        # Run pick and place operations
        self.pick_object()
        self.place_object()

if __name__ == '__main__':
    rclpy.init(args=None)
    node = UR5ePickAndPlaceNode()
    node.run()
    rclpy.shutdown()