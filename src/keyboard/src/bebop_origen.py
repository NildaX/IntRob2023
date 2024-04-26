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
                position = {'x': -0.006339543795801471,
                        'y': -0.060178817351622366,
                        'z': 0.050735410511438525}

                # Define the orientation
                orientation = {'x': -6.013997907795954e-06,
                            'y': -4.0184256807335854e-05,
                            'z': -0.11672072169886474,
                            'w': 0.9931647755912174}

                # Create a Pose message
                self.pose_msg = Pose()

                # Set the position
                self.pose_msg.position.x = position['x']
                self.pose_msg.position.y = position['y']
                self.pose_msg.position.z = position['z']

                # Set the orientation
                self.pose_msg.orientation.x = orientation['x']
                self.pose_msg.orientation.y = orientation['y']
                self.pose_msg.orientation.z = orientation['z']
                self.pose_msg.orientation.w = orientation['w']
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
            reset_pose_msg.pose = self.pose_msg

            # Assuming reset_pose_publisher is your ROS publisher
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
