### Explanation

1. **Node Initialization**: The node is initialized with a name `arm_controller`.

2. **Parameter Definition**: The script sets the initial and target positions for the arm.

3. **Publisher Setup**: A publisher is set up to publish Twist messages, which are used to control the robot's arm.

4. **Timer Setup**: A timer is created to periodically publish the setpoints at regular intervals. This simulates a continuous transition between the two specified positions.

5. **Setpoint Interpolation**: The script uses linear interpolation to smoothly transition between the initial and target positions.

### Step 4: Run the Script

Run the script using the following command: