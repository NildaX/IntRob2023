
import rospy
from std_srvs.srv import Empty

def reset_simulation():
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_simulation_proxy()
        rospy.loginfo("Simulation reset complete.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('reset_simulation_node', anonymous=True)
    reset_simulation()

