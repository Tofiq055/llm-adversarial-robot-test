import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from ur_msgs.srv import MoveJointTrajectoryGoal
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

class UR5eController():
    def __init__(self):
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        rospy.Service('/move_group/execute_trajectory', MoveJointTrajectoryGoal, self.execute_trajectory)
        rospy.spin()

    def execute_trajectory(self, request):
        # Your trajectory vector array
        waypoints = [
            PoseStamped(pose=Pose(position=[0.3, -0.2, 0.4]), orientation=Quaternion(x=0, y=0, z=0, w=1)),
            PoseStamped(pose=Pose(position=[0.3, 0.2, 0.4]), orientation=Quaternion(x=0, y=0, z=0, w=1)),
            PoseStamped(pose=Pose(position=[0.4, 0.0, 0.3]), orientation=Quaternion(x=0, y=0, z=0, w=1))
        ]
        request.trajectory = waypoints
        return WaypointGoalResponse()

    def joint_state_callback(self, data):
        # Just for printing the robot state while moving
        print("Joint states: ", data)

def main():
    rospy.init_node('UR5eController', anonymous=True)
    controller = UR5eController()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass