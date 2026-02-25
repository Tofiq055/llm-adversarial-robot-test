import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist, Vector3
from ur_modern_driver.srv import SetCartesianMode, SetCartesianModeRequest, SetCartesianModeResponse
from ur_modern_driver.srv import MoveJointTrajectory, MoveJointTrajectoryRequest

def move_to_objective(x, y, z, roll, pitch, yaw):
    # Check if the robot arm is already in Cartesian mode
    rospy.wait_for_service('cartesian_mode')
    cartesian_mode = rospy.ServiceProxy('cartesian_mode', SetCartesianMode)
    
    try:
        response = cartesian_mode(1)  # Enable Cartesian mode
        
        # Move the end-effector to the objective coordinates and orientation
        end_effector_pose = PoseStamped()
        end_effector_pose.header.stamp = rospy.Time.now()
        end_effector_pose.header.frame_id = "ur5e_end_effector"
        end_effector_pose.pose.position.x = x
        end_effector_pose.pose.position.y = y
        end_effector_pose.pose.position.z = z
        end_effector_pose.pose.orientation.w = 1.0
        
        cartesian_goal = Vector3()
        cartesian_goal.x = x
        cartesian_goal.y = y
        cartesian_goal.z = z
        
        # Convert the orientation quaternion to a rotation matrix and apply it to the end-effector pose
        rotation_matrix = np.array([[np.cos(pitch), -np.sin(pitch), 0], [np.sin(pitch), np.cos(pitch), 0], [0, 0, 1]])
        
        rotation_matrix = apply_rotation_matrix(rotation_matrix)
        
        # Move the end-effector to the desired position and orientation
        move_to_pose(end_effector_pose.pose.position.x, end_effector_pose.pose.position.y, end_effector_pose.pose.position.z, roll, pitch, yaw)
        
    except rospy.ServiceException as e:
        print("Failed to enable Cartesian mode: ", str(e))
    
def apply_rotation_matrix(matrix):
    # Apply the rotation matrix to the end-effector pose
    # The rotation matrix is applied to the end-effector pose in the order of X, Y, Z
    x = end_effector_pose.pose.position.x
    y = end_effector_pose.pose.position.y
    z = end_effector_pose.pose.position.z
    
    rotated_x = matrix[0][0] * x + matrix[1][0] * y + matrix[2][0] * z
    rotated_y = matrix[0][1] * x + matrix[1][1] * y + matrix[2][1] * z
    rotated_z = matrix[0][2] * x + matrix[1][2] * y + matrix[2][2] * z
    
    end_effector_pose.pose.position.x = rotated_x
    end_effector_pose.pose.position.y = rotated_y
    end_effector_pose.pose.position.z = rotated_z
    
    return end_effector_pose

def move_to_pose(x, y, z, roll, pitch, yaw):
    # Move the end-effector to the desired position and orientation
    rospy.wait_for_service('move_joint_trajectory')
    move_joint_trajectory = rospy.ServiceProxy('move_joint_trajectory', MoveJointTrajectory)
    
    try:
        # Create a twist message for the end-effector orientation
        orientation_twist = Twist()
        orientation_twist.angular.z = yaw
        
        # Create a joint trajectory message with the desired position and orientation
        joint_trajectory = MoveJointTrajectoryRequest()
        joint_trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        joint_trajectory.points.append(TrajectoryPoint())
        joint_trajectory.points[0].positions = [x, y, z, roll, pitch, 0]
        joint_trajectory.points[0].velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_trajectory.points[0].accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Move the end-effector to the desired position and orientation
        response = move_joint_trajectory(joint_trajectory)
    
    except rospy.ServiceException as e:
        print("Failed to move to the pose: ", str(e))

if __name__ == '__main__':
    rospy.init_node('move_to_objective', anonymous=True)
    x = 0.3
    y = 0.2
    z = 0.4
    roll = 0.0
    pitch = 3.14
    yaw = 0.0
    
    move_to_objective(x, y, z, roll, pitch, yaw)