import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from ur_robot_driver.srv import MoveToPose
from std_srvs.srv import Empty

class Ur5eTrajectoryNode(Node):
    def __init__(self):
        super().__init__('ur5e_trajectory_node')
        
        # Create a client for the MoveToPose service
        self.move_to_pose_client = self.create_client(MoveToPose, '/moveit_commander/execute_planning_scene')
        
        # Check if the client connection was successful
        while not self.move_to_pose_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for MoveToPose service...')
        
        # Create a service client to stop the robot's motion
        self.stop_motion_client = self.create_client(Empty, '/ur_driver/stop')
        
        # Check if the client connection was successful
        while not self.stop_motion_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for StopMotion service...')
        
        # Initialize the trajectory points
        self.points = [
            PoseStamped(
                header=self.get_clock().now().to_msg(),
                pose=Pose(
                    position=Point(x=0.3, y=-0.2, z=0.4),
                    orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
                )
            ),
            PoseStamped(
                header=self.get_clock().now().to_msg(),
                pose=Pose(
                    position=Point(x=0.3, y=0.2, z=0.4),
                    orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
                )
            ),
            PoseStamped(
                header=self.get_clock().now().to_msg(),
                pose=Pose(
                    position=Point(x=0.4, y=0.0, z=0.3),
                    orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
                )
            )
        ]
        
        # Initialize the current point index
        self.current_point_index = 0
        
        # Start the trajectory execution
        self.execute_trajectory()
    
    def execute_trajectory(self):
        while not self.current_point_index >= len(self.points):
            # Stop any existing motion to start a new one
            try:
                response = self.stop_motion_client.call_async(Empty.Request())
            except Exception as e:
                self.get_logger().error(f"Failed to stop motion: {e}")
            
            # Move to the next point in the trajectory
            pose = self.points[self.current_point_index]
            try:
                future = self.move_to_pose_client.call_async(MoveToPose.Request(pose=pose))
                
                def callback(response):
                    if response.success:
                        self.get_logger().info(f"Reached point {self.current_point_index + 1}")
                        self.current_point_index += 1
                        self.execute_trajectory()
                    else:
                        self.get_logger().error("Failed to move to the next pose")
            
                future.add_done_callback(callback)
            except Exception as e:
                self.get_logger().error(f"Failed to execute trajectory: {e}")
    
    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = Ur5eTrajectoryNode()
    node.run()

if __name__ == '__main__':
    main()