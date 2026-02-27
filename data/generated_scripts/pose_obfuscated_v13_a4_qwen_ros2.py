import rclpy
from rclpy.node import Node
from ur_msgs.msg import JointCommand, FollowJointTrajectoryActionGoal

class UR5eBacklashNode(Node):
    def __init__(self):
        super().__init__('ur5e_backlash_node')
        
        # Create a publisher for joint commands
        self.publisher = self.create_publisher(JointCommand, 'controller_joint/command', 10)
        
        # Define the target position and orientation
        self.target_position = [0.3, 0.2, 0.4]
        self.target_orientation = [0.0, 3.14, 0.0]
        
        # Create a goal for the joint trajectory action
        self.goal = FollowJointTrajectoryActionGoal()
        self.goal.goal.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
        
        # Set the target positions and orientations
        for i, joint in enumerate(['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']):
            self.goal.goal.points[0].positions.append(self.target_position[i])
        
        self.goal.goal.points[0].velocities = [0.0] * 6
        self.goal.goal.points[0].accelerations = [0.0] * 6
        
        # Set the goal tolerance
        self.goal.goal.tolerance.x = 0.01
        self.goal.goal.tolerance.y = 0.01
        self.goal.goal.tolerance.z = 0.01
        self.goal.goal.tolerance.heading = 0.1
        
    def run(self):
        rclpy.spin_once(self)
        
        # Send the goal to the action server
        self.publisher.publish(self.goal)

def main(args=None):
    rclpy.init(args=args)
    
    node = UR5eBacklashNode()
    try:
        node.run()
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()