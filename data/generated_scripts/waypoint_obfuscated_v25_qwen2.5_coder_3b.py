import rclpy
from ur_robot_driver.srv import SetJoints

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('trajectory_executor')

    # Define the trajectory waypoints
    waypoints = [
        [0.3, -0.2, 0.4],
        [0.3, 0.2, 0.4],
        [0.4, 0.0, 0.3]
    ]

    # Set up the rate for publishing joint positions
    rate = rclpy.create_rate(10)

    while rclpy.ok():
        for waypoint in waypoints:
            # Create a service client to set the joints
            client = node.create_client(SetJoints, '/ur_driver/set_joints')
            if not client.wait_for_service(timeout_sec=1.0):
                node.get_logger().error('Service not available')

            request = SetJoints.Request()
            request.position = waypoint

            # Call the service to set joint positions
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)

        rate.sleep()

    rclpy.shutdown()

if __name__ == '__main__':
    main()