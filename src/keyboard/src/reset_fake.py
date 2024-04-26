import roslaunch
import rospy
import subprocess
import time
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates,ModelState
def run_launch_file():
    # Initialize the ROS node
    rospy.init_node('launch_file_runner')

    # Specify the path to your launch file
    #launch_file_path = "/home/nilda/IntRob2023/src/rotors_simulator/rotors_gazebo/launch/test_bebop.launch"
    launch_file_path = "/home/nilda/IntRob2023/src/rotors_simulator/rotors_gazebo/launch/test_solofake.launch"

    # Create a launch object
    #--
    
    cont=0
    cont_max=2
    while cont<cont_max:
        try:
            roslaunch_process = subprocess.Popen(['roslaunch', launch_file_path])
            rospy.loginfo("Launch file started successfully")
            # Wait for the launch file to finish
            # Monitor the status of the launch process
            while roslaunch_process.poll() is None:
                rospy.sleep(1)  # Sleep for 1 second and check again
            rospy.loginfo("Launch file finished")
        except roslaunch.RLException as e:
            rospy.logerr("Failed to start launch file: %s" % str(e))
        finally:
            # Shutdown the launch object
            #launch.shutdown()
            # Shutdown the ROS node
            rospy.signal_shutdown("Finished running launch file")
        cont+=1
    print("----------finaliza-------------")

if __name__ == '__main__':
    run_launch_file()


