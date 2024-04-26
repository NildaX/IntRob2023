import rospy
from gazebo_msgs.srv import DeleteModel
from std_srvs.srv import Empty
import subprocess
import os
def reset_node(node_name):
    print("reset node")
    rospy.wait_for_service('/rosnode/kill')
    try:
        kill_node = rospy.ServiceProxy('/rosnode/kill', Empty)
        kill_node(node_name)
        rospy.loginfo("Node '{}' reset successfully.".format(node_name))
    except rospy.ServiceException as e:
        rospy.logerr("Failed to reset node '{}': {}".format(node_name, e))
        
def delete_model(model_name):
    print("delete model")
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
    print("main")
    # Replace 'model_name_to_delete' with the name of the model you want to delete
    
    model_name_to_delete = 'bebop2'
     
    success = delete_model(model_name_to_delete)
    
    if success:
        print("Model '{}' deleted successfully.".format(model_name_to_delete))
        #os.system("rosnode kill /bebop2/fake_driver") 
        launch_file_path = "/home/nilda/IntRob2023/src/rotors_simulator/rotors_gazebo/launch/put_bebop_in_world.launch"
        
        roslaunch_process = subprocess.Popen(['roslaunch', launch_file_path])
        rospy.loginfo("Launch file started successfully")
        roslaunch_process.wait()
        
        print("Finished running launch file")
       
        
    else:
        print("Failed to delete model '{}'.".format(model_name_to_delete))
    print("----------finaliza-------------")

