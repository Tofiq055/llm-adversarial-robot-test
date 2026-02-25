import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from ur5e_control.srv import MoveArm
from math import pi

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')
        
        # Create a service client for moving the arm
        self.move_arm_client = self.create_client(MoveArm, 'move_arm')
        while not self.move_arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the move_arm service...')
        
        # Define the target pose
        target_pose = Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.2
        target_pose.position.z = 0.4
        
        target_orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=pi,  # Roll is 0
            w=1.0   # Pitch and Yaw are pi (180 degrees)
        )
        
        target_pose.orientation = target_orientation
        
        # Create a joint state message to specify the arm positions
        joint_state = JointState()
        joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint',
                           'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                           'wrist_3_joint']
        
        # Initialize the joint state with zero values
        for i in range(len(joint_state.name)):
            joint_state.position.append(0.0)
        
        # Move the arm to the target pose
        self.move_arm_request = MoveArm.Request()
        self.move_arm_request.pose = target_pose
        self.move_arm_request.joint_state = joint_state
        
        self.future = self.move_arm_client.call_async(self.move_arm_request)

    def move_callback(self, future):
        try:
            result = future.result()
            if result.successful:
                self.get_logger().info('Arm moved successfully to target pose.')
            else:
                self.get_logger().error('Failed to move arm: %s' % result.reason)
        except Exception as e:
            self.get_logger().exception('Exception while calling the move_arm service: %s' % e)

def main(args=None):
    rclpy.init(args=args)
    node = UR5eMoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()