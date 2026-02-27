import rospy
from moveit_msgs.msg import PlanningScene, PlannerResponse, MoveGroupActionGoal, Trajectory, Constraints
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatusArray
import sys
import math
import time
import moveit_commander as mc
import trac_ik_py as ik

# Set up the MoveIt! interface
rospy.init_node('move_ur5e', anonymous=True)
robot = mc.RobotCommander()
scene = mc.PlanningSceneCommander()
group_name = "ur5e"  # Set this to the name of your MoveIt! group (usually 'ur5e')
group = mc.MoveGroupCommander(group_name)

# Set the robot and planning scene
robot.set_planning_time(10)
scene.clear()

# Add collision objects to the scene, if needed
# For example:
# box = Box(width=0.2, height=0.3, depth=0.5)
# box_pose = PoseStamped()
# box_pose.header.frame_id = "world"
# box_pose.pose.position = Point(0.4, 0, -0.1)
# scene.add_box("Box", box, box_pose)

# Set the planning group to use collision checking
group.set_planning_frame("base_link")
group.allow_collisions = True

# Plan a Cartesian path from a start point to an end point
start_point = PoseStamped()
start_point.header.frame_id = "world"
start_point.pose.position = Point(0, 0, -0.3)

end_points = [
    PoseStamped(),
    PoseStamped(),
    PoseStamped(),
]
end_points[0].header.frame_id = "world"
end_points[0].pose.position = Point(0.3, -0.2, 0.4)
end_points[1].header.frame_id = "world"
end_points[1].pose.position = Point(0.3, 0.2, 0.4)
end_points[2].header.frame_id = "world"
end_points[2].pose.position = Point(0.4, 0, 0.3)

# Set the planning group's maximum joint velocities and accelerations (in m/s^2 and m/s^2)
group.set_max_velocity_scaling_factor(0.2)
group.set_max_acceleration_scaling_factor(0.2)

# Plan a Cartesian path between the start point and end points using MoveIt!
joint_trajectory = group.plan(start_point, end_points, eef_pose_tolerance=0.1)

# Execute the planned trajectory with smooth interpolation
group.execute(joint_trajectory)