import rclpy
from rclpy.node import Node
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose

class UR5eCalibrationTest(Node):
    def __init__(self):
        super().__init__('ur5e_calibration_test')

        # Initialize the robot commander
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()

        # Create a move group for the UR5e arm
        self.move_group = MoveGroupCommander('manipulator')

        # Set initial pose (optional)
        initial_pose = Pose()
        initial_pose.position.x = 0.0
        initial_pose.position.y = 0.0
        initial_pose.position.z = 0.0
        initial_pose.orientation.w = 1.0
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_pose_target(initial_pose)
        self.move_group.go(wait=True)

        # Define the trajectory vectors
        trajectory_vectors = [
            [0.3, -0.2, 0.4],
            [0.3, 0.2, 0.4],
            [0.4, 0.0, 0.3]
        ]

        # Iterate through each trajectory vector and move the robot
        for vector in trajectory_vectors:
            pose = Pose()
            pose.position.x = vector[0]
            pose.position.y = vector[1]
            pose.position.z = vector[2]
            self.move_group.set_pose_target(pose)
            self.move_group.go(wait=True)

if __name__ == '__main__':
    rclpy.init(args=None)
    test_node = UR5eCalibrationTest()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()