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
from sensor_msgs.msg import Imu, Range

import numpy as np
import math

from geometry_msgs.msg import Twist

from nav_msgs.msg import Path
import geometry_msgs.msg as gm

from pyquaternion import Quaternion
from numpy import linalg as LA

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class bebop_control:
	def __init__(self):
		self.bridge = CvBridge()
		self.rgbImg = rospy.Subscriber('/bebop2/front_camera/rgb/image_raw', Image, self.callback, queue_size=1, buff_size=2**24)
		self.Gazebo = rospy.Subscriber('/gazebo/model_states', ModelStates, self.Gazebo_pose)
		self.Imu_ = rospy.Subscriber('/bebop2/imu', Imu, self.imu)
		self.sonar_ = rospy.Subscriber('/bebop2/sonar', Range, self.sonar)
		self.Ovr = rospy.Subscriber('/keyboard/override', Int8, self.flag)
		
		self.pub_cmd_vel = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=10)
		self.vel_msg = Twist()
		self.path = rospy.Publisher('/bebop2/path', Path, queue_size=1, latch=True)
		
		self.robot_name = 'bebop2'
		
		self.Wpt_x = rospy.get_param("Wpts/x")
		self.Wpt_y = rospy.get_param("Wpts/y")
		self.Wpt_z = rospy.get_param("Wpts/z")
		
		self.kp_pitch = rospy.get_param("pitch/kp")
		self.kp_yaw = rospy.get_param("yaw/kp")
		self.kp_alt = rospy.get_param("altitude/kp")
		
		print('========== From config.yaml file ========== ')
		print("Wpt_x", self.Wpt_x)
		print("Wpt_y", self.Wpt_y)
		print("Wpt_z", self.Wpt_z)
		print('')
		print("kp_pitch", self.kp_pitch)
		print("kp_yaw", self.kp_yaw)
		print("kp_alt", self.kp_alt)
		
		self.override = 0
		self.state = 0
		
		rot_e = 0.0
		rot_u = 0.0
		
		dist_e = 0.0
		dist_u = 0.0
	
	def callback(self,data):
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		
		if (self.override == 1):
			
			if (self.state == 0):
				
				print('====================================')
				print ('State: ', self.state)
				print('====================================')
				
				self.pr = np.array([self.Wpt_x, self.Wpt_y])
				self.p1 = np.array([self.x, self.y])
				
				self.nvec = self.pr - self.p1
				
				self.dist_ref = np.linalg.norm(self.nvec)
				
				self.yaw_ref = (math.atan2(self.nvec[1], self.nvec[0]))* (180.0/math.pi)
				
				maxAng = 90.0 * math.pi / 180.0
				rot_e = self.yaw_ref - self.yaw
				angle = rot_e / maxAng
				rot_u = angle * self.kp_yaw
				
				print ('')
				print("angle ref: ",self.yaw_ref)
				print("angle current: ",self.yaw)
				print("angle error: ", rot_e)
				print ('')
				
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = rot_u
				
				if(abs(rot_e) < 0.2):
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = 0.0
					
					self.p0 = np.array((self.x, self.y))
					print(self.p0)
					self.state = 1
					
			
			if(self.state == 1):
				
				print('====================================')
				print ('State: ', self.state)
				print('====================================')
				
				self.p = np.array((self.x, self.y))
				dist = np.linalg.norm(self.p0 - self.p)
				dist_e = self.dist_ref - dist
				dist_u = dist_e * self.kp_pitch
				
				print ('')
				print("dist ref: ",self.dist_ref)
				print("dist current: ", dist)
				print("dist error: ", dist_e)
				print ('')
				
				self.vel_msg.linear.x = dist_u
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
			
			self.pub_cmd_vel.publish(self.vel_msg)
			
		# ~ cv2.imshow('Image Viewer - RGB', frame)
		# ~ cv2.waitKey(1)

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
		
		q = (self.qw, self.qx, self.qy, self.qz)
		self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(q)
		# radians to degree
		self.yaw = (self.yaw * 180.0)/math.pi
		
		self.display_path(self.x, self.y, self.z, self.qw, self.qx, self.qy, self.qz)
		
		
		

	def imu(self,imu_msg):
		self.imu = imu_msg.orientation
		# ~ imu_q = (self.imu.x, self.imu.y, self.imu.z, self.imu.w)
		# ~ self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(imu_q)
		# ~ # radians to degree
		# ~ self.roll = (self.roll * 180.0)/math.pi
		# ~ self.pitch = (self.pitch * 180.0)/math.pi
		# ~ self.yaw = (self.yaw * 180.0)/math.pi

	def sonar(self,sonar_msg):
		self.altitude = sonar_msg.range

	def display_path(self, x, y, z, qw, qx, qy, qz):
		msg = Path()
		msg.header.frame_id = "world"
		msg.header.stamp = rospy.Time(0)
		
		robotPath = gm.PoseStamped()
		robotPath.pose.position = gm.Point(x, y, z)
		robotPath.pose.orientation = gm.Quaternion(qw,qx,qy,qz)
		
		msg.poses.append(robotPath)
		self.path.publish(msg)

		
	def flag(self,msg):
		# ~ print ("Override: ", msg.data)
		
		if msg.data == 6:
			self.override = 1
		else:
			self.override = 0
			self.state = 0
			rot_e = 0.0
			rot_u = 0.0
			
			dist_e = 0.0
			dist_u = 0.0
		

def main():
	rospy.init_node('bebop_control', anonymous = True)
	print("init Node Position Controller")
	bebop_control()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()
		sys.exit()

if __name__ == '__main__':
	main()
