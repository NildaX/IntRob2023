import rospy
from gazebo_msgs.srv import DeleteModel
import subprocess
def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = delete_model_proxy(model_name)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed:", e)
        return False

if __name__ == "__main__":
    rospy.init_node('delete_model_node')
    
    # Replace 'model_name_to_delete' with the name of the model you want to delete
    model_name_to_delete = 'bebop2'

    success = delete_model(model_name_to_delete)
    if success:
        print("Model '{}' deleted successfully.".format(model_name_to_delete))
        launch_file_path = "/home/nilda/IntRob2023/src/rotors_simulator/rotors_gazebo/launch/test_bebop.launch"
        roslaunch_process = subprocess.Popen(['roslaunch', launch_file_path])
        rospy.loginfo("Launch file started successfully")
        rospy.signal_shutdown("Finished running launch file")
        
    else:
        print("Failed to delete model '{}'.".format(model_name_to_delete))
    print("----------finaliza-------------")

