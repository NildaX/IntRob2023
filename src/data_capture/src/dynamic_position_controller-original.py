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
		
		
		#Esto expresa la velocidad en el espacio libre dividida en sus partes lineales y angulares
		self.pub_cmd_vel = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=10)
		self.vel_msg = Twist()
		#Una matriz de poses que representa un Camino para que lo siga un robot
		self.path = rospy.Publisher('/bebop2/path', Path, queue_size=1, latch=True)
		#A Pose con marco de coordenadas de referencia y marca de tiempo
		self.rviz = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_pose)
		#El mensaje de marcador se utiliza para enviar "marcadores" de visualización como cuadros, esferas, flechas, líneas, etc. a un entorno de visualización como rviz.
		self.wpts_pub = rospy.Publisher('/wpts', Marker, queue_size = 10)
		
		self.robot_name = 'bebop2'
		
		self.kp_pitch = rospy.get_param("pitch/kp")
		self.kp_yaw = rospy.get_param("yaw/kp")
		self.kp_alt = rospy.get_param("altitude/kp")
		
		##---agregado
		self.kd_yaw = rospy.get_param("yaw/kd")
		self.ki_yaw = rospy.get_param("yaw/ki")
		self.ki_pitch = rospy.get_param("pitch/ki")
		self.kd_pitch = rospy.get_param("pitch/kd")
		
		print('========== From config.yaml file ========== ')
		print("kp_pitch", self.kp_pitch)
		print("kp_yaw", self.kp_yaw)
		print("kp_alt", self.kp_alt)
		##--agregado
		print("kd_yaw", self.kd_yaw)
		print("ki_yaw", self.ki_yaw)
		
		##---
		
		self.override = 0
		self.state = 0
		
		self.goal_point = np.array([0.0, 0.0, 0.0])
		
		self.Wpts = []
		self.new_pose = False
	
	def callback(self,data):
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		
		if (self.override == 1):
			###---------estado controlador proporcional para corregir la orientación
			if (self.state == 0):
				
				print('====================================')
				print ('Dynamic position State: ', self.state)
				print('====================================')
				
				print ('Nav goal: ', self.goal_point)
				#pr es el objetivo a llegar
				self.pr = self.goal_point
				#p1 es la posicion actual del dron
				self.p1 = np.array([self.x, self.y, self.z])
				#distancia entre el objetivo y la distancia actual
				self.nvec = self.pr - self.p1
				#obtiene distancia euclidiana de la distancia obtenida anteriormente
				self.dist_ref = np.linalg.norm(self.nvec)
				#yaw es el eje z, entiendo que saca el arco tangente para saber como llegar a ese lugar en grados
				self.yaw_ref = (math.atan2(self.nvec[1], self.nvec[0]))* (180.0/math.pi)
				
				#hace calculos para obtener el angulo de referencia, el angulo actual, angulo error,  
				maxAng = 90.0 * math.pi / 180.0
				rot_e = self.yaw_ref - self.yaw
				angle = rot_e / maxAng
				
				
				
				##--- este es el control proporcional para calcular la orientacion
				rot_u = angle * self.kp_yaw
				
				##---------------------------------------------PID
				Ts=0.01
				ei=0
				e_1=0
				e_1=angle
				print(angle)
				ep=(angle-e_1)/Ts
				u=rot_u + (self.ki_yaw*ei) + (self.kd_yaw*ep)
				ei=ei+angle*Ts
				e_1=angle
				print(e_1)
				print(u)
				#rot_i = angle * self.ki_yaw
				##-----derivativo
				#rot_d = angle * self.kd_yaw
				
				#-----------------------------------------------PID
				#rot_pid=rot_u+rot_i+rot_d
				print ('')
				print("angle ref: ",self.yaw_ref)
				print("angle current: ",self.yaw)
				print("angle error: ", rot_e)
				print("PID: ", u)
				print ('')
				
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = u#rot_u#rot_pid###
				#cambiar esta condicion, verifica el angulo de rotacion, si es menor a 0.2 para cambiar a estado 1
				
				
				if(abs(rot_e) <= 0.1):
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = 0.0
					
					self.p0 = np.array((self.x, self.y,self.z))
					print(self.p0)
					self.state = 1
			
			if(self.state == 1):
				##segundo estado implementó un controlador proporcional para cambiar la altura del dron
				print('====================================')
				print ('Dynamic position State: ', self.state)
				print('====================================')
				
				dist_ref_altura=2.0
				dist_altura=dist_ref_altura-self.z
				dist_alturac = dist_altura * self.kp_alt
				
				#falta la referencia de altura
				## para z?
				#dist_al = dist_e * self.kp_alt
				
				## para z?
				#dist_z = dist_e * self.kp_alt
				
				
				
				print("dist altura: ",dist_ref_altura)
				print("dist altura real: ", self.z)
				print("dist error: ", dist_altura)
				
				print ('')
				self.vel_msg.linear.x = 0.0#dist_u
				self.vel_msg.linear.y = 0.0#dist_al#0.0 #------------
				self.vel_msg.linear.z = dist_alturac#0.0#dist_z#0.0 #------------este para abajo o arriba
				self.vel_msg.angular.z = 0.0
				if(abs(dist_altura) < 0.2):
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = 0.0
					
					self.p0 = np.array((self.x, self.y,self.z))
					print(self.p0)
					self.state = 2
				
				
			if(self.state == 2):
				print('====================================')
				print ('Dynamic position State: ', self.state)
				print('====================================')
				#obtiene actual x y y---------- agregar z?
				
				
				self.p = np.array((self.x, self.y,self.z))
				
				#compara p actual con p0 que entiendo es como a posicion anterior-del estado 0
				dist = np.linalg.norm(self.p0 - self.p)
				#obtiene distancias 
				dist_e = self.dist_ref - dist
				#---este es el control proporcional para desplazar al dron a x,y
				dist_u = dist_e * self.kp_pitch
				##---------------------------------------------PID pitch x
				Ts=0.01
				ei=0
				e_1=0
				e_1=dist_e
				print(dist_e)
				ep=(dist_e-e_1)/Ts
				u=dist_u + (self.ki_pitch*ei) + (self.kd_pitch*ep)
				ei=ei+dist_e*Ts
				e_1=dist_e
				#print(e_1)
				#print(u)
				#rot_i = angle * self.ki_yaw
				##-1----derivativo
				#rot11_d = angle * self.kd_yaw
				print ('')
				print("dist ref: ",self.dist_ref)
				print("dist current: ", dist)
				print("dist error: ", dist_e)
				self.vel_msg.linear.x = dist_u
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				
				#-----------------------------------------------PID
			if(self.new_pose):
				print("Dynamic position new pose")
				#si es una nueva pose resetea las posiciones
				print("updating goal point: ",self.goal_point)
				
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				
				rot_e = 0.0
				rot_u = 0.0
				
				dist_e = 0.0
				dist_u = 0.0
				self.new_pose = False
				self.state = 0
				
				
					
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


	def display_waypoint(self,Wpts):
		points = Marker()
		points.header.frame_id = "world"
		points.type = points.SPHERE
		points.action = points.ADD
		
		self.marker_id = 1
		
		points.scale.x = .25
		points.scale.y = .25
		points.scale.z = .25
		points.color.a = 1
		
		points.color.r = 0.0
		points.color.g = 1.0
		points.color.b = 0.0
		
		points.pose.orientation.x = 0.0
		points.pose.orientation.y = 0.0
		points.pose.orientation.z = 0.0
		points.pose.orientation.w = 1.0
		
		for p in Wpts:
			points.id = self.marker_id
			self.marker_id += 1
			points.pose.position.x = p[0]
			points.pose.position.y = p[1]
			points.pose.position.z = p[2]
			
			self.wpts_pub.publish(points)
	
	def rviz_pose(self,pose_msg):
		x = pose_msg.pose.position.x
		y = pose_msg.pose.position.y
		z = pose_msg.pose.position.z
		
		print("pose_msg.pose.position: ",pose_msg.pose.position)
		
		self.Wpts.append([x,y,z])
		
		self.goal_point[0] = x
		self.goal_point[1] = y
		self.goal_point[2] = z
		
		print("updating goal point: ",self.goal_point)
		self.display_waypoint(self.Wpts)
		self.new_pose= True
		
	def flag(self,msg):
		# ~ print ("Override: ", msg.data)
		
		if msg.data == 5:
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
