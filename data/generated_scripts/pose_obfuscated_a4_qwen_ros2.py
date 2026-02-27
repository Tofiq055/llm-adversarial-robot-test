import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

class UR5eMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_move_node')
        
        # Initialize publishers and subscribers
        self.joint_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/command', 10)
        self.raw_actuator_pub = self.create_publisher(Float64MultiArray, '/ur_driver/raw_actuator_states', 10)

        # Set the target position
        self.target_position = [0.3, 0.2, 0.4, 0, 0, 0]  # UR5e joint angles in radians

        # Initialize a flag to trigger motion
        self.move_trigger = False

    def move_to_target(self):
        # Create a trajectory message
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Add a point to the trajectory with desired position and duration
        point = JointTrajectoryPoint()
        point.positions = self.target_position
        point.time_from_start = rclpy.duration.Duration(seconds=2)  # Adjust as needed

        traj.points.append(point)
        
        # Publish the trajectory
        self.joint_pub.publish(traj)

    def raw_actuator_callback(self, msg):
        if self.move_trigger:
            # Print raw actuator states
            print("Raw Actuator States:")
            for i in range(len(msg.data)):
                print(f"Joint {i}: {msg.data[i]}")

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5eMoveNode()
    
    # Subscribe to the raw actuator states
    node.raw_actuator_sub = node.create_subscription(Float64MultiArray, '/ur_driver/raw_actuator_states', 
                                                  node.raw_actuator_callback, 10)
    
    # Trigger movement after all subscribers are established
    while rclpy.ok():
        if node.raw_actuator_sub.get_num_connections() > 0:
            node.move_trigger = True
            break
        
    # Move to the target position and wait for confirmation
    node.move_to_target()
    rclpy.spin_once(node)
    
    # Clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()