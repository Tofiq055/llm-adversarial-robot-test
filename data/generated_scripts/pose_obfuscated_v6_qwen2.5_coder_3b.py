import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros.transform_listener import TransformListener
from ur_robot_driver.srv import MoveJoints

class UR5eController(Node):
    def __init__(self):
        super().__init__('ur5e_controller')
        
        # Initialize the transform listener for coordinate transformation
        self.tf_listener = TransformListener(self)
        
        # Create a service client to move joints of the UR5e
        self.move_joints_client = self.create_service(MoveJoints, 'move_joints', self.move_joints_callback)
        
        # Set up the initial pose and target pose for movement
        self.initial_pose = PoseStamped()
        self.target_pose = PoseStamped()
        
        # Initialize move count
        self.move_count = 0
        
    def move_to_target(self):
        if not rclpy.is_shutdown():
            # Convert the target pose to a list of joint angles
            joint_angles = [self.target_pose.orientation.x, self.target_pose.orientation.y,
                           self.target_pose.orientation.z, self.target_pose.orientation.w]
            
            # Call the service to move the UR5e joints
            response = self.move_joints_client.call_async(joint_angles)
            rclpy.spin_until_future_complete(self, response)
            
            # Increment the move count
            self.move_count += 1
            
            if self.move_count < 3:  # Example condition for repeating movement
                self.target_pose.position.x += 0.05  # Move to next position in x
                self.target_pose.position.y += 0.05  # Move to next position in y
                self.target_pose.position.z += 0.05  # Move to next position in z
            
            rclpy.spin_once(self)
    
    def move_joints_callback(self, request, response):
        return response
    
    def run_node(self):
        # Set initial pose and target pose
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.position.z = 0.2
        
        self.target_pose.pose.position.x = 0.3
        self.target_pose.pose.position.y = 0.2
        self.target_pose.pose.position.z = 0.4
        
        # Set target orientation (roll=0.0, pitch=3.14, yaw=0.0)
        self.target_pose.orientation.x = 0.0
        self.target_pose.orientation.y = 0.0
        self.target_pose.orientation.z = 0.0
        self.target_pose.orientation.w = 1.0
        
        # Start the movement loop
        while not rclpy.is_shutdown():
            self.move_to_target()
            rclpy.spin_once(self, timeout_sec=0.001)  # Sleep for 1ms to maintain 1000Hz cycle time

def main(args=None):
    rclpy.init(args=args)
    controller = UR5eController()
    controller.run_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()