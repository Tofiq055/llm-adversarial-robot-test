import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from moveit_commander import MoveGroupCommander
from math import radians

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        
        # Initialize the move group for the UR5e robot
        self.move_group = MoveGroupCommander('arm', node=self)
        
        # Set the goal position and orientation
        target_pose = Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.2
        target_pose.position.z = 0.4
        
        target_orientation = Quaternion(
            x=0.0,
            y=radians(3.14),
            z=0.0,
            w=1.0
        )
        
        target_pose.orientation = target_orientation
        
        # Move the robot arm to the goal position and orientation
        self.move_group.set_pose_target(target_pose)
        
        # Plan and execute the motion
        success = self.move_group.go(wait=True)
        
        if success:
            print("Move successful!")
        else:
            print("Move failed!")

def main(args=None):
    rclpy.init(args=args)
    
    robot_arm_controller = RobotArmController()
    
    try:
        rclpy.spin(robot_arm_controller)
    except KeyboardInterrupt:
        pass
    
    robot_arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()