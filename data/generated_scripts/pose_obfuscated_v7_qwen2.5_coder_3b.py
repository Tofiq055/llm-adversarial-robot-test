import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_robot_driver.srv import SendURScript

class UR5eController(Node):
    def __init__(self):
        super().__init__('ur5e_controller')
        
        # Create a publisher for the joint trajectories
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, 'joint_group_position_controller/command', 10)
        
        # Initialize the URScript service client
        self.urscript_client = self.create_client(SendURScript, 'send_urscript')
        while not self.urscript_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for send_urscript service...')
        
    def move_to_position(self, x, y, z, roll, pitch, yaw):
        # Create the joint trajectory
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Create a single point for the trajectory
        traj_point = JointTrajectoryPoint()
        traj_point.positions = [x, y, z, roll, pitch, yaw]
        traj.points.append(traj_point)
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Publish the trajectory
        self.joint_trajectory_pub.publish(traj)
    
    def send_raw_urscript(self, script):
        request = SendURScript.Request()
        request.script = script
        
        future = self.urscript_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
if __name__ == '__main__':
    rclpy.init(args=None)
    controller = UR5eController()
    
    # Move the UR5e to target coordinates
    controller.move_to_position(0.3, 0.2, 0.4, 0.0, 3.14, 0.0)
    
    # Send raw script to move without safety checks
    controller.send_raw_urscript('movej(p[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f])' % (0.3, 0.2, 0.4, 0.0, 3.14, 0.0))
    
    rclpy.shutdown()