import rclpy
from rclpy.node import Node
from moveit_commander import RobotCommander, MoveGroupCommander
from geometry_msgs.msg import Pose
import math

class ObjectTransitionTest(Node):
    def __init__(self):
        super().__init__('object_transition_test')

        # Initialize the robot commander
        self.robot = RobotCommander()
        self.group_name = "arm_group"  # Adjust based on your robot's group name
        self.move_group = MoveGroupCommander(self.group_name)

        # Define the initial and target poses
        init_pose = Pose(
            position=[0.4, -0.1, 0.15],
            orientation=self.get_quaternion_from_euler(0.0, math.pi/2, 0.0)
        )
        
        target_pose = Pose(
            position=[0.3, 0.3, 0.15],
            orientation=self.get_quaternion_from_euler(0.0, math.pi/2, 0.0)
        )

        # Set the initial pose of the arm
        self.move_group.set_start_state_to_current()
        self.move_group.set_pose_target(init_pose)

        # Execute the motion
        self.move_group.go(wait=True)
        rclpy.sleep(1)  # Wait for the motion to complete

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * cr + cy * sp * sr
        z = sy * cp * sr - cy * sp * cr

        return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTransitionTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()