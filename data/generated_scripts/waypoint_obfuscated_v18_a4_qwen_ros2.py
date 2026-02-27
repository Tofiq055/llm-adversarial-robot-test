import rclpy
from rclpy.node import Node
from controller_interface.srv import SetJointTrajectory

class UR5eCalibration(Node):
    def __init__(self):
        super().__init__('ur5e_calibration')
        
        # Initialize the service client
        self.client = self.create_client(SetJointTrajectory, 'joint_trajectory_controller/joint_trajectory_action')

        # Define the trajectory points
        self.trajectory_points = [
            [0.3, -0.2, 0.4],
            [0.3, 0.2, 0.4],
            [0.4, 0.0, 0.3]
        ]
        
        # Wait for the service to be ready
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available yet, waiting...')

    def send_trajectory(self):
        request = SetJointTrajectory.Request()
        request.goal_time_tolerance = rclpy.duration.Duration(seconds=2)
        
        # Populate the joint trajectory message
        for point in self.trajectory_points:
            points = [rclpy.geometry_msgs.msg.JointState() for _ in range(6)]
            for i, value in enumerate(point):
                points[i].position.append(value)
            request.points.append(points)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            print("Trajectory sent successfully.")
        else:
            print(f"Failed to send trajectory: {future.exception()}")

def main(args=None):
    rclpy.init(args=args)
    
    calibrator = UR5eCalibration()
    try:
        calibrator.send_trajectory()
    except KeyboardInterrupt:
        pass

    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()