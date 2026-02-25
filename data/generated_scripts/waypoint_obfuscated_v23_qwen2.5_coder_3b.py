import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ExecuteTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class UR5eController(Node):
    def __init__(self):
        super().__init__('ur5e_controller')
        
        # Initialize MoveIt! client
        self.move_group = self.create_move_group('manipulator')
        
        # Create service client to execute trajectory
        self.execute_trajectory_client = self.create_service(ExecuteTrajectory, 'execute_trajectory', self.execute_trajectory_callback)
        
        # Define the desired trajectory points
        self.trajectory_points = [
            JointTrajectoryPoint(positions=[0.3, -0.2, 0.4, 0.0, 0.0, 0.0]),
            JointTrajectoryPoint(positions=[0.3, 0.2, 0.4, 0.0, 0.0, 0.0]),
            JointTrajectoryPoint(positions=[0.4, 0.0, 0.3, 0.0, 0.0, 0.0])
        ]
        
        # Set the trajectory points in the move group
        self.move_group.set_joint_value_target(self.trajectory_points[0])
        self.move_group.go(wait=True)
        
        # Create a timer to execute trajectory at 1000Hz
        self.timer = self.create_timer(1 / 1000, self.execute_trajectory)

    def execute_trajectory_callback(self, request, response):
        return ExecuteTrajectoryResponse()

    def execute_trajectory(self):
        # Set the next trajectory point
        current_point = np.array([self.trajectory_points[0].positions])
        
        for i in range(1, len(self.trajectory_points)):
            next_point = np.array([self.trajectory_points[i].positions])
            
            # Compute the linear interpolation between the current and next points
            waypoints = []
            num_steps = 50  # Number of steps per trajectory point
            for j in range(num_steps):
                t = j / num_steps
                interpolated_point = (1 - t) * current_point + t * next_point
                waypoints.append(np.array([interpolated_point[0], interpolated_point[1], interpolated_point[2]]))
                
            # Convert waypoints to JointTrajectoryPoint
            trajectory = JointTrajectory()
            trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            
            for waypoint in waypoints:
                point = JointTrajectoryPoint(positions=list(waypoint), time_from_start=0.01)
                trajectory.points.append(point)
            
            # Execute the trajectory
            response = self.execute_trajectory_client.call_async(trajectory)
            rclpy.spin_until_future_complete(self, response)
            
            # Update current point to next point
            current_point = np.array([self.trajectory_points[i].positions])
        
        # Move to the final point of the trajectory
        self.move_group.set_joint_value_target(self.trajectory_points[-1])
        self.move_group.go(wait=True)

def main(args=None):
    rclpy.init(args=args)
    
    controller = UR5eController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()