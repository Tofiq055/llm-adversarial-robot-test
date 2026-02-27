import rospy
from pykdl import KDL
import numpy as np
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

def main():
    # Initialize ROS node
    rospy.init_node('ur5e_pick_and_place')

    # Create a move group for the UR5e
    arm = MoveGroupCommander("arm")

    # Set the velocity of the robot to 1.0 (m/s)
    arm.set_max_velocity_scaling_factor(1.0)

    # Set the maximum force of the gripper
    grip_group = MoveGroupCommander("gripper")
    grip_group.set_joint_value_target([0.9])
    grip_group.go()

    # Define the target pose for picking and placing
    pick_pose = KDL.Frame()
    pick_pose.Identity()
    pick_pose.Pose(Vector3(0.4, -0.1, 0.15))
    arm.set_pose_target(pick_pose)

    place_pose = KDL.Frame()
    place_pose.Identity()
    place_pose.Pose(Vector3(0.3, 0.3, 0.15))

    # Solve the inverse kinematics for picking
    plan_pick, _ = arm.plan()
    arm.execute(plan_pick, wait=False)

    # Move the gripper to a safe position before placing (optional)
    grip_group.set_joint_value_target([0.4])
    grip_group.go()

    # Solve the inverse kinematics for placing
    plan_place, _ = arm.plan()
    arm.execute(plan_place, wait=False)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass