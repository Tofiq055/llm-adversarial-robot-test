# Import necessary packages
import rospy
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped

# Initialize Move Group and Planning Scene
planning_scene_interface = PlanningSceneInterface()
move_group = MoveGroupCommander("ur5e")

# Set the trajectory vector array
trajectory = [[0.3, -0.2, 0.4], [0.3, 0.2, 0.4], [0.4, 0.0, 0.3]]

def execute_trajectory():
    for i in range(len(trajectory)):
        # Set joint values to the current trajectory point
        move_group.set_joint_value_target([trajectory[i][0], trajectory[i][1], trajectory[i][2]])
        
        # Block until the movement is finished
        move_group.go()

if __name__ == "__main__":
    rospy.init_node("ur5e_trajectory", anonymous=True)
    execute_trajectory()