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

class bebop_control:
	def __init__(self):
		self.bridge = CvBridge()
		self.rgbImg = rospy.Subscriber('/bebop2/front_camera/rgb/image_raw', Image, self.callback, queue_size=1, buff_size=2**24)
		self.Gazebo = rospy.Subscriber('/gazebo/model_states', ModelStates, self.Gazebo_pose)
		self.Imu_ = rospy.Subscriber('/bebop2/imu', Imu, self.imu)
		self.sonar_ = rospy.Subscriber('/bebop2/sonar', Range, self.sonar)
		self.Ovr = rospy.Subscriber('/keyboard/override', Int8, self.flag)
		
		self.path = rospy.Publisher('/bebop2/path', Path, queue_size=1, latch=True)
		self.pub_cmd_vel = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=10)
		self.vel_msg = Twist()
		
		self.robot_name = 'bebop2'
		
		self.deg_ref = rospy.get_param("degrees")
		self.alt_ref = rospy.get_param("altitude")
		self.dist_ref = rospy.get_param("distance")
		
		self.kp_rot = rospy.get_param("Orien_gains/kp")
		self.kp_alt = rospy.get_param("alt_gains/kp")
		self.kp_dist = rospy.get_param("dist_gains/kp")
		
		print('========== From config.yaml file ========== ')
		print("degrees", self.deg_ref)
		print("altitude", self.alt_ref)
		print("distance", self.dist_ref)
		print('')
		print("kp_rot", self.kp_rot)
		print("kp_alt", self.kp_alt)
		print("kp_dist", self.kp_dist)
		
		rot_e = 0.0
		rot_u = 0.0
		alt_e = 0.0
		alt_u = 0.0
		dist_e = 0.0
		dist_u = 0.0
				
		self.override = 0
		self.state = 0
	
	def callback(self,data):
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		
		if (self.override == 1):
			
			
			if(self.state == 0):
				print('====================================')
				print ('Bebop control State: ', self.state)
				print('====================================')
			
				maxAng = 90.0 * math.pi / 180.0
				rot_e = self.deg_ref - self.yaw
				angle = rot_e / maxAng
				rot_u = angle * self.kp_rot
				
				print ('')
				print("angle ref: ",self.deg_ref)
				print("angle current: ",angle)
				print("angle error: ", rot_e)
				print ('')
				
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = rot_u
				
				if(abs(rot_e) < 0.2):
					
					self.vel_msg.linear.x = 0.0 #pitch inclinacion hacia el frente
					self.vel_msg.linear.y = 0.0 #roll inclinacion en lateral derecha negativo izquerda positivo
					self.vel_msg.linear.z = 0.0 #altitude
					self.vel_msg.angular.z = 0.0 #yaw positivo izquerda
					
					self.state = 1
					
			
			if(self.state == 1):
				print('====================================')
				print ('Bebop control State: ', self.state)
				print('====================================')
				
				alt_e = self.alt_ref - self.altitude
				alt_u = alt_e * self.kp_alt
				
				print ('')
				print("alt ref: ",self.alt_ref)
				print("alt current: ",self.altitude)
				print("alt error: ", alt_e)
				print ('')
				
				self.vel_msg.linear.x = 0.0 #pitch
				self.vel_msg.linear.y = 0.0 #roll
				self.vel_msg.linear.z = alt_u #alt
				self.vel_msg.angular.z = 0.0 #yaw
				
				if(abs(alt_e) < 0.1):
					
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = 0.0
					
					self.p0 = np.array((self.x, self.y))
					
					self.state = 2
					
			if(self.state == 2):
				
				print('====================================')
				print ('Bebop control State: ', self.state)
				print('====================================')
				
				self.p1 = np.array((self.x, self.y)) #distancia euclidiana dist_u
				dist = np.linalg.norm(self.p0 - self.p1)
				dist_e = self.dist_ref - dist
				dist_u = dist_e * self.kp_dist
				
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
		
		cv2.imshow('Image Viewer - RGB', frame)
		cv2.waitKey(1)

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
		
		self.display_path(self.x, self.y, self.z, self.qw, self.qx, self.qy, self.qz)

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

	def display_path(self, x, y, z, qw, qx, qy, qz): #posicion x y z y orientacion
		msg = Path()
		msg.header.frame_id = "world"
		msg.header.stamp = rospy.Time(0)
		
		robotPath = gm.PoseStamped()
		robotPath.pose.position = gm.Point(x, y, z)
		robotPath.pose.orientation = gm.Quaternion(qw, qx, qy, qz)
		
		msg.poses.append(robotPath)
		self.path.publish(msg)


	def flag(self,msg):
		# ~ print ("Override: ", msg.data)
		
		if msg.data == 5: #tecla 5 para evitar la acumulacion de error
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
	print("init Node Bebop Control")
	bebop_control()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()
		sys.exit()

if __name__ == '__main__':
	main()
