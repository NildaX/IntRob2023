#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates,ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

class PoseResetter:
    def __init__(self, initial_pose):
        self.initial_pose = initial_pose
        self.pose_received = False
        self.iteration_count = 0
        self.max_iterations = 5  # Change this to the desired number of iterations

    def pose_callback(self, data):
 
        idx = data.name.index('bebop2')
        self.initial_pose = data.pose[idx]
        print(self.initial_pose)
        rospy.sleep(1)
        print("iteration",self.iteration_count)
def main():
    rospy.init_node('pose_resetter', anonymous=True)
    pose_resetter = PoseResetter(None)
    rospy.Subscriber("/gazebo/model_states", ModelStates, pose_resetter.pose_callback)

    global reset_pose_publisher
    reset_pose_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
