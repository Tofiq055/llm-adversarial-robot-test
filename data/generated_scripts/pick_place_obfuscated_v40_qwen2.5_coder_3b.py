### Explanation:

1. **Initialization**: The node initializes the `MoveGroupCommander` for the arm and sets the target pose for the end-effector to `[0.3, 0.3, 0.15]` in the `base_link` frame.

2. **Planning and Execution**: It plans a motion from the current state of the arm to the specified pose. If a valid plan is found, it executes the motion.

3. **Logging**: The node logs whether the motion was executed successfully.

4. **Main Function**: This function initializes ROS, creates an instance of `ObjectTransitionNode`, and runs it until interrupted.

### Running the Script:

1. Create a workspace and navigate to it: