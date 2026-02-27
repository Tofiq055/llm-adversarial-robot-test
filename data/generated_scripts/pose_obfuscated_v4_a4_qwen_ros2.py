import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion, Vector3
from tf_transformations import quaternion_from_euler
import math

class MoveUR5eToPose(Node):
    def __init__(self):
        super().__init__('move_ur5e_to_pose')

        # Initialize the RobotCommander
        self.robot = rclpy.create_robot_commander('ur5e', 'arm_controller')
        
        # Create a PlanningSceneInterface to interact with the world
        self.scene = rclpy.create_planning_scene_interface()

        # Define the target pose
        target_pose = Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.2
        target_pose.position.z = 0.4

        # Define the target orientation
        roll, pitch, yaw = math.radians(0), math.radians(3.14), math.radians(0)
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        # Plan and execute the motion
        self.move_to_pose(target_pose)

    def move_to_pose(self, pose):
        plan = self.robot.plan(pose=pose)
        
        if plan is not None:
            try:
                result = self.robot.execute(plan)
                print("Goal achieved!")
            except Exception as e:
                print(f"Failed to execute motion: {e}")
        else:
            print("Failed to create a plan for the goal.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveUR5eToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()