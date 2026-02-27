class UR5eTrajectoryExecutor(Node):
       def __init__(self):
           super().__init__('ur5e_trajectory_executor')
           
           # Initialize publishers and subscribers
           self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
           self.joint_state_subscriber_ = self.create_subscription(
               JointState,
               'joint_states',
               self.joint_state_callback,
               10)
           
           # Create a transform broadcaster for the UR5e's base frame
           self.br = TransformBroadcaster()
           
           # Set initial joint positions (assuming zero initially)
           self.initial_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
           
           # Define the trajectory waypoints
           self.waypoints = [
               [0.3, -0.2, 0.4],
               [0.3, 0.2, 0.4],
               [0.4, 0.0, 0.3]
           ]
           
           # Initialize current waypoint index
           self.current_waypoint_index = 0
           
           # Create a FollowJointTrajectoryGoal
           self.goal = FollowJointTrajectoryGoal()
           self.goal.trajectory = JointTrajectory(joint_names=['shoulder_pan_joint', 'shoulder_lift_joint',
                                                              'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                                              'wrist_3_joint'])
           
           # Set the initial joint positions
           for i, pos in enumerate(self.initial_joint_positions):
               point = JointTrajectoryPoint(positions=[pos])
               self.goal.trajectory.points.append(point)
           
           # Start executing the trajectory
           self.execute_trajectory()
       
       def execute_trajectory(self):
           while not rclpy.is_shutdown():
               # Set the joint positions for the current waypoint
               for i, pos in enumerate(self.waypoints[self.current_waypoint_index]):
                   point = JointTrajectoryPoint(positions=[pos])
                   self.goal.trajectory.points[i].positions[0] = pos
                   
               # Publish the trajectory goal to the action server
               self.client_.send_goal_async(self.goal)
               
               # Wait for the trajectory execution to complete
               result = rclpy.action.Future()
               self.client_.wait_for_result(result)
               
               if result.result().status == 'succeeded':
                   self.current_waypoint_index += 1
                   print(f"Waypoint {self.current_waypoint_index} executed")
                   
                   # Break the loop if all waypoints are reached
                   if self.current_waypoint_index >= len(self.waypoints):
                       break
               
               else:
                   print("Trajectory execution failed")
           
           rclpy.spin_until_future_complete(self.client_, result)
       
       def joint_state_callback(self, msg):
           # Update the robot's current joint positions
           self.initial_joint_positions = [msg.position[i] for i in range(len(msg.position))]