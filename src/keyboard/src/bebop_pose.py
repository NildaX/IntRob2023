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
        if not self.pose_received:
            try:
                idx = data.name.index('bebop2')
                self.initial_pose = data.pose[idx]
                self.pose_received = True
                rospy.loginfo("Initial Pose Saved:\n{}".format(self.initial_pose))
            except ValueError:
                rospy.logwarn("Robot not found in ModelStates message.")

        if self.pose_received and self.iteration_count < self.max_iterations:
            # Do something in each iteration (e.g., move the robot)
            # For demonstration, I'm just printing the iteration count
            rospy.loginfo("Iteration count: {}".format(self.iteration_count))
            self.iteration_count += 1
        elif self.iteration_count >= self.max_iterations:
            reset_pose_msg = ModelState()
            reset_pose_msg.model_name = 'bebop2'
            reset_pose_msg.pose = self.initial_pose
            reset_pose_publisher.publish(reset_pose_msg)
            rospy.loginfo("Robot pose reset to initial pose.")
        rospy.sleep(0.5)
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
