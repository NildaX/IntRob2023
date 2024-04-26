#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates,ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkState
from std_msgs.msg import Float32
from mav_msgs.msg import Actuators
class PoseResetter:
    def __init__(self, initial_pose):
        self.initial_pose = initial_pose
        self.pose_received = False
        self.iteration_count = 0
        self.max_iterations = 3  # Change this to the desired number of iterations

    def pose_callback(self, data):
        if not self.pose_received:
            position = {'x': -0.006339543795801471,
                        'y': -0.060178817351622366,
                        'z': 0.050735410511438525}

            # Define the orientation
            orientation = {'x': -6.013997907795954e-06,
                        'y': -4.0184256807335854e-05,
                        'z': -0.11672072169886474,
                        'w': 0.9931647755912174}
            #twist = {'linear': {x: 0.0, y: 0.0, z: 0.0 }, 'angular':{x: 0.0, y:0.0, z: 0.0 }}

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
            


            # Create a JointState message
            self.joint_state_msg = JointState()

            # Fill in the header information
            self.joint_state_msg.header.seq = 9504
            self.joint_state_msg.header.stamp.secs = 54240
            self.joint_state_msg.header.stamp.nsecs = 85000000
            self.joint_state_msg.header.frame_id = "bebop2/base_link"

            # Fill in the names of the joints
            self.joint_state_msg.name = ['bebop2/rotor_0_joint',
                                    'bebop2/rotor_1_joint',
                                    'bebop2/rotor_2_joint',
                                    'bebop2/rotor_3_joint']

            # Fill in the positions of the joints
            self.joint_state_msg.position = [0.11178374736511021,
                                        0.1117837473429768,
                                        0.1117837473736536,
                                        0.11178374736176355]

            
            

        if self.pose_received and self.iteration_count < self.max_iterations:
            # Do something in each iteration (e.g., move the robot)
            # For demonstration, I'm just printing the iteration count
            rospy.loginfo("Iteration count: {}".format(self.iteration_count))
            self.iteration_count += 1
        elif self.iteration_count >= self.max_iterations:
            # Assuming self.initial_pose is the Pose message created above
            reset_pose_msg = ModelState()
            reset_pose_msg.model_name = 'bebop2'
            reset_pose_msg.pose = self.pose_msg

            # Assuming reset_pose_publisher is your ROS publisher
            reset_pose_publisher.publish(reset_pose_msg)
            rospy.sleep(1)
            print("Robot pose reset to initial pose.")

            joint_state_publisher.publish(self.joint_state_msg)
            motor_speed = 0  # Motor speed value
            pub_motor_0.publish(motor_speed)
            pub_motor_1.publish(motor_speed)
            pub_motor_2.publish(motor_speed)
            pub_motor_3.publish(motor_speed)
            msg = Actuators()
            msg.angular_velocities = [0.0, 0.0, 0.0, 0.0]  # Fill this array with your desired motor speeds
            pub_motor_speed.publish(msg)

            print("termino")
            rospy.sleep(1)


        rospy.sleep(0.5)
        print("iteration",self.iteration_count)
def main():
    rospy.init_node('pose_resetter', anonymous=True)
    pose_resetter = PoseResetter(None)
    rospy.Subscriber("/gazebo/model_states", ModelStates, pose_resetter.pose_callback)

    global reset_pose_publisher
    reset_pose_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    global joint_state_publisher
    joint_state_publisher = rospy.Publisher('/bebop2/joint_states', JointState, queue_size=10)
    global link_state_publisher, pub_motor_0, pub_motor_1, pub_motor_2, pub_motor_3, pub_motor_speed
    link_state_publisher = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
    pub_motor_0 = rospy.Publisher('bebop2/motor_speed/0', Float32, queue_size=10)
    pub_motor_1 = rospy.Publisher('bebop2/motor_speed/1', Float32, queue_size=10)
    pub_motor_2 = rospy.Publisher('bebop2/motor_speed/2', Float32, queue_size=10)
    pub_motor_3 = rospy.Publisher('bebop2/motor_speed/3', Float32, queue_size=10)
    pub_motor_speed = rospy.Publisher('/bebop2/command/motor_speed', Actuators, queue_size=10)


    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
