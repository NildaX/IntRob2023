#!/usr/bin/env python
__author__ = "L. Oyuki Rojas-Perez and Dr. Jose Martinez-Carranza"
__license__ = "GPL"
__version__ = "1.0.1"
__maintainer__ = "L. Oyuki Rojas-Perez"
__email__ = "carranza@inaoep.mx"

import rospy
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8

from gazebo_msgs.msg import ModelStates
import tf
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu, Range
import numpy as np
import math
from geometry_msgs.msg import Twist

class data_acquisition:
	def __init__(self):
		self.bridge = CvBridge()
		self.rgbImg = rospy.Subscriber('/bebop2/front_camera/rgb/image_raw', Image, self.callback, queue_size=1, buff_size=2**24)
		self.depthImg = rospy.Subscriber('/bebop2/front_camera/depth/image_raw', Image, self.depth_image, queue_size=1, buff_size=2**24)
		self.Gazebo = rospy.Subscriber('/gazebo/model_states', ModelStates, self.Gazebo_pose)

		self.Imu_ = rospy.Subscriber('/bebop2/imu', Imu, self.imu)
		self.sonar_ = rospy.Subscriber('/bebop2/sonar', Range, self.sonar)
		self.Ovr = rospy.Subscriber('/keyboard/override', Int8, self.flag)
		
		self.pub_cmd_vel = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=10)
		self.vel_msg = Twist()
		
		self.robot_name = 'bebop2'
		
		self.override = 0
		self.u_alt=0.0
	
	def callback(self,data):
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		
		print (' ')
		print (' ')
		print ('				Gazebo model name: ' + self.robot_name)
		print ('-------------------------------------------------------------------------------')
		print ('		Pose			|		Orientation')
		print ('-------------------------------------------------------------------------------')
		print ('	x:', self.x, '		|	qx:', self.qx) 
		print ('	y:', self.y, '		|	qy:', self.qy) 
		print ('	z:', self.z, '		|	qz:', self.qz) 
		print ('						|	qw:', self.qw)
		print (' ')
		print ('-------------------------------------------------------------------------------')
		print ('				IMU')
		print ('-------------------------------------------------------------------------------')
		print ('	qx:', self.imu.x, '		|	roll:', self.roll)
		print ('	qy:', self.imu.y, '		|	pitch:', self.pitch)
		print ('	qz:', self.imu.z, '		|	yaw:', self.yaw)
		print ('	qw:', self.imu.w)
		print (' ')
		print ('-------------------------------------------------------------------------------')
		print ('				Sonar')
		print ('-------------------------------------------------------------------------------')
		print ('	altitude:', self.altitude)
			
		cv2.imshow('Image Viewer - RGB', frame)
		# ~ cv2.imshow("Image Viewer - Depth", self.depthimg)
		cv2.waitKey(1)

	def depth_image(self,msg_depth):
		
		# The depth image is a single-channel float32 image
		# the values is the distance in mm in z axis
		cv_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")
		# Convert the depth image to a Numpy array since most cv2 functions
		# require Numpy arrays.
		cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
		# Normalize the depth image to fall between 0 (black) and 1 (white)
		self.depthimg = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
		# Resize to the desired size

	def Gazebo_pose(self,data):
		model_states = data
		model_names = model_states.name
		index = model_names.index(self.robot_name)
		
		self.x = model_states.pose[index].position.x;
		self.y = model_states.pose[index].position.y;
		self.z = model_states.pose[index].position.z;
		
		self.qw = model_states.pose[index].orientation.x;
		self.qx = model_states.pose[index].orientation.y;
		self.qy = model_states.pose[index].orientation.z;
		self.qz = model_states.pose[index].orientation.w;

	def imu(self,imu_msg):
		self.imu = imu_msg.orientation
		imu_q = (self.imu.x, self.imu.y, self.imu.z, self.imu.w)
		self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(imu_q)
		# radians to degree
		self.roll = (self.roll * 180.0)/math.pi
		self.pitch = (self.pitch * 180.0)/math.pi
		self.yaw = (self.yaw * 180.0)/math.pi

	def sonar(self,sonar_msg):
		self.altitude = sonar_msg.range
		
	def flag(self,msg):
		print ("Override: ", msg.data)
		
		if msg.data == 5:
			self.override = 1
		else:
			self.override = 0
		

def main():
	rospy.init_node('data_acquisition', anonymous = True)
	print("init Node Data Acquisition")
	data_acquisition()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()
		sys.exit()

if __name__ == '__main__':
	main()
