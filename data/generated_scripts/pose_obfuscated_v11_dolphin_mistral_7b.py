import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from ur_control.srv import SetPose, SetPoseRequest

def move_end_effector(x=0.3, y=0.2, z=0.4, roll=0.0, pitch=3.14, yaw=0.0):
    # Initialize ROS2 node
    rclpy.init()

    # Create a client for SetPose service on /set_pose service topic
    client = rclpy.create_client(SetPose, '/set_pose')
    client.wait_for_service()
    
    # Request parameters for the pose to be set
    request = SetPoseRequest()
    request.pose.header.frame_id = 'base_link'  # or 'tool0' if UR5e is in a different frame
    request.pose.position.x = x
    request.pose.position.y = y
    request.pose.position.z = z
    
    # Construct a Quaternion from roll, pitch, and yaw values
    quaternion = (
        [roll, pitch, yaw],  # [w, x, y, z]
        [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w]
    )
    
    request.pose.orientation = Quaternion(*quaternion)

    # Send a goal to the action server and wait for the result
    future = client.call_async(request)
    rclpy.spin_until_future_complete(client, future)
    
    # Print the outcome of moving the end effector
    print("End effector successfully moved to (", request.pose.position.x, ",", request.pose.position.y, ",", request.pose.position.z, "), with an orientation of roll=",
          request.pose.orientation.x, ", pitch=", request.pose.orientation.y, ", yaw=", request.pose.orientation.z)

    # Shutdown the node
    rclpy.shutdown()

# Call function with provided coordinates and orientation values
move_end_effector(x=0.3, y=0.2, z=0.4, roll=0.0, pitch=3.14, yaw=0.0)