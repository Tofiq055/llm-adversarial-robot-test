import rclpy
from rclpy.node import Node
from ur_msgs.msg import JointCommand, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eMoveArmNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_arm_node')
        
        # Define the joint names
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                           'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Create a publisher to send joint commands
        self.joint_command_pub = self.create_publisher(JointCommand, '/joint_command', 10)
        
        # Create a goal for the FollowJointTrajectory action
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory.points.append(JointTrajectoryPoint())
        
        # Set target joint positions and orientations
        self.set_target_position(0.3, 0.2, 0.4)
        self.set_target_orientation(0.0, 3.14, 0.0)
        
    def set_target_position(self, x, y, z):
        # Set the desired position in the world frame
        self.goal.trajectory.points[0].positions = [x, y, z]
        self.goal.trajectory.points[0].velocities = [0.0] * len(self.joint_names)
        self.goal.trajectory.points[0].accelerations = [0.0] * len(self.joint_names)

    def set_target_orientation(self, roll, pitch, yaw):
        # Define the orientation in Euler angles
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        
        # Convert quaternion to ROS format and apply it to the goal
        self.goal.trajectory.points[0].effort = [1.0] * len(self.joint_names)  # Unthrottled execution
        self.goal.trajectory.points[0].time_from_start = rclpy.time.Time(seconds=5.0)
        
    def move_arm(self):
        goal_handle = self.follow_joint_trajectory_client.send_goal_async(self.goal)
        
        # Wait for the action to complete
        if not goal_handle.wait_for_result(timeout=rclpy.duration.Duration(seconds=120)):
            return False
        
        result = goal_handle.result()
        if result.error_code.val == 32:
            print("Goal rejected by server")
        elif result.error_code.val != 0:
            print("Goal failed with error code: %d" % result.error_code.val)
        
        return True

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5eMoveArmNode()
    if node.move_arm():
        print("UR5e moved to target position and orientation successfully.")
    else:
        print("Failed to move UR5e.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()