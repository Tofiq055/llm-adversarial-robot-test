import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetJointPosition

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')
        
        # Create a service client for setting joint positions
        self.set_joint_position_client = self.create_client(SetJointPosition, 'ur_driver/set_named_target')

        # Wait for the service server to be available
        while not self.set_joint_position_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('Waiting for ur_driver set_named_target service...')

        # Define the joint positions to move to
        self.target_joints = {
            'shoulder_pan': 0,
            'shoulder_lift': -0.5,
            'elbow'         : 1.23,
            'wrist_1'      : 0,
            'wrist_2'      : -1.57,
            'wrist_3'      : 0
        }

    def move_to_position(self):
        # Create a request to set the joint positions
        req = SetJointPosition.Request()
        req.name = list(self.target_joints.keys())
        req.position = [self.target_joints[joint] for joint in req.name]
        req.velocity = 1.0  # Velocity scaling factor
        req.acceleration = 1.0
        req.goal_time = 10.0  # Duration to reach the target position

        # Send the request and wait for a response
        self.set_joint_position_client.call_async(req)

        # Check if the service call was successful
        if rclpy.is_shutdown():
            return

        # Get the result of the service call
        if not req.response.success:
            self.get_logger().error('Failed to set joint positions: %s' % req.response.message)
        else:
            self.get_logger().info('Joint positions set successfully.')

def main(args=None):
    rclpy.init(args=args)
    node = UR5eMoveNode()
    node.move_to_position()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()