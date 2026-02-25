Replace `<rosdistro>` with your ROS2 distribution (e.g., `foxy`).

### Step 1: Create a MoveIt! Robot Model

Create or modify the UR5e robot model in MoveIt!. You can do this by copying the UR5e description from `ur_description` and editing it to include the harmonic drive backlash information.

### Step 2: Implement the Abrupt Direction Reversal Constraint

Create a custom plugin to enforce abrupt direction reversals. This plugin will modify the trajectory to ensure that certain joints are reversed without changing their position.