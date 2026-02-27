import rclpy
from rclpy.node import Node
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5eBacklashMeasurement(Node):
    def __init__(self):
        super().__init__('ur5e_backlash_measurement')
        
        # Initialize the controller client
        self.controller_client = self.create_subscription(
            FollowJointTrajectoryGoal,
            'joint_trajectory_controller/follow_joint_trajectory/goal',
            self.callback,
            10)
    
    def callback(self, goal):
        # Extract target joint positions from the goal
        target_positions = []
        for joint in goal.trajectory.points[0].positions:
            target_positions.append(joint)
        
        # Set target end-effector pose
        x = 0.3
        y = 0.2
        z = 0.4
        
        roll = 0.0
        pitch = 3.14
        yaw = 0.0
        
        # Convert Euler angles to quaternion
        import math
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        
        # Set the joint positions and orientation
        target_positions[0] = x  # Position along x-axis (should be directly above base)
        target_positions[1] = y  # Position along y-axis (should be to the right of base)
        target_positions[2] = z  # Position along z-axis (should be towards end-effector)
        target_positions[3] = qx
        target_positions[4] = qy
        target_positions[5] = qz
        target_positions[6] = qw
        
        # Create a goal object for the controller
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.points.append(JointTrajectoryPoint(joint_names=self.get关节_names(), positions=target_positions))
        
        # Send the goal to the controller
        self.controller_client.send_goal(goal)
        
    def get_joint_names(self):
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        return joint_names

def main(args=None):
    rclpy.init(args=args)
    node = UR5eBacklashMeasurement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()