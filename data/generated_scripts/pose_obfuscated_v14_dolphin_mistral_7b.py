import rclpy
from robot_msgs.srv import MoveArm
from geometry_msgs.msg import Pose

class MoveRobotArm(rclpy.Node):
    def __init__(self):
        super().__init__('move_robot_arm')
        self.goal_publisher = self.create_publisher(Pose, 'goal_pose', 10)
        self.client = MoveArm()
        
    def move_to_target(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        
        target_pose.orientation.w = 1.0
        target_pose.orientation.x = roll
        target_pose.orientation.y = pitch
        target_pose.orientation.z = yaw
        
        self.goal_publisher.publish(target_pose)
        self.client.call_async('/move', target_pose).wait_for_response()

    def move_arm_to_coordinates(self):
        # Define the coordinates and orientation for the end-effector
        x = 0.3
        y = 0.2
        z = 0.4
        roll = 0.0
        pitch = 3.14
        yaw = 0.0
        
        # Move the robot arm to the specified coordinates and orientation
        self.move_to_target(x, y, z, roll, pitch, yaw)
        
def main():
    rclpy.init()
    move_robot_arm = MoveRobotArm()
    move_robot_arm.move_arm_to_coordinates()
    rclpy.shutdown()

if __name__ == '__main__':
    main()