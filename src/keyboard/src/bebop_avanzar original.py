#!/usr/bin/env python3
__author__ = "L. Oyuki Rojas-Perez and Dr. Jose Martinez-Carranza"
__license__ = "GPL"
__version__ = "1.0.1"
__maintainer__ = "L. Oyuki Rojas-Perez"
__email__ = "carranza@inaoep.mx"

import rospy
import sys
from getkey import getkey, keys
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
import time
class keyboard:
	def __init__(self):
		self.pub_takeoff = rospy.Publisher('/bebop2/takeoff', Empty, queue_size=10)
		self.pub_land = rospy.Publisher('/bebop2/land', Empty, queue_size=10)
		self.pub_cmd_vel = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=10)
		self.pub_override = rospy.Publisher('/keyboard/override', Int8, queue_size=10)
		self.vel_msg = Twist()
		self.OvR_msg = 0
		self.speed_value = 0.3
		
		self.init_msg = """		-------------------------------
		Increment speed	- Right arrow
		Decrement speed	- Left arrow
		
		Stop	- H
		Takeoff		- T
		Land		- Space bar
		
		Forward		- W
		Backward	- S
		
		Right		- D
		Left		- A
		
		Ascend		- Up arrow
		Descend		- Down arrow
		
		Rotate (Right)	- E
		Rotate (Left)	- Q
		
		Autonomous	- X
		Manual		- C
		
		"""
		
		self.final_msg = """		-------------------------------
		Press CTRL + C and then enter to quit
		"""
		
		
		
		self.VALUE_SECTION=0
		self.VALUE_DISTANCE=0

		self.VALUE_CENTRO=99

		self.VALUE_OBJETOS=[]
		self.VALUE_OBJETOS_DISTANCES=[]
		self.VALUE_OBJETOS_SECTIONS=[]

		self.Ovr = rospy.Subscriber('/camera/section', Int8, self.flag)
		self.Dista = rospy.Subscriber('/camera/distance', Float64, self.flag2)
		#--
		self.centro = rospy.Subscriber('/camera/distance_centro', Float64, self.flag3)
		#--
		self.objetcs_detected = rospy.Subscriber('/camera/objects', Int32MultiArray, self.flag4)
		self.objetcs_detected_distances = rospy.Subscriber('/camera/objects_distances', Float32MultiArray, self.flag5)
		self.objetcs_detected_sections = rospy.Subscriber('/camera/objects_sections', Int32MultiArray, self.flag6)

		self.class_name_dict = {0: 'table', 1: 'chair', 2: 'sofa', 3: 'fridge', 4: 'barbell', 5: 'bed', 6: 'tv', 7: 'ball', 8: 'cabinet', 9: 'glass', 10: 'portrait', 11: 'door', 12: 'shelf', 13: 'wall'}
		self.class_name_sections = {0: 'CENTER',1:'TOP LEFT',2:'TOP RIGHT',3:'BOTTOM LEFT',4:'BOTTOM RIGHT'}
		self.class_name_actions = {'a': 'LEFT','w':'FORWARD','s':'STOP','d':'RIGHT','x':'REVERSE'}
		self.cmd_vel()
		
		#self.key='T'
		#self.key_anterior=self.key
	def flag(self,msg):
		self.VALUE_SECTION=msg.data
	def flag2(self,msg):
		self.VALUE_DISTANCE=msg.data
	def flag3(self,msg):
		self.VALUE_CENTRO=msg.data
	def flag4(self,msg):
		self.VALUE_OBJETOS=msg.data
	def flag5(self,msg):
		self.VALUE_OBJETOS_DISTANCES=msg.data
	def flag6(self,msg):
		self.VALUE_OBJETOS_SECTIONS=msg.data
	

	def cmd_vel(self):
		key='T'
		while not rospy.is_shutdown():
			
			key_anterior=key
			if key == 'RIGHT':
				self.speed_value +=0.05
				if self.speed_value >= 1.0:
					self.speed_value = 1.0
				print('		Last Command Send: Increment speed')
				
			if key == 'LEFT':
				self.speed_value -=0.05
				if self.speed_value <= 0.0:
					self.speed_value = 0.0
				print('		Last Command Send: Decrement speed')
				
			## Takeoff
			if key == 'T':
				msg = Empty()
				self.pub_takeoff.publish(msg)
				print('		Last Command Send: Takeoff')
				
			## Land
			if key == 'SPACE':
				msg = Empty()
				self.pub_land.publish(msg)
				print('		Last Command Send: Land')
				
			## Ascend
			if key == 'UP':
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = round(self.speed_value,2)
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				print('		Last Command Send: Ascend')
				
			## Descend
			if key == 'DOWN':
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = -round(self.speed_value,2)
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				print('		Last Command Send: Descend')
				
			## Translate forward 
			if key == 'W':
				self.vel_msg.linear.x = round(self.speed_value,2)
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				print('		Last Command Send: Forward')
				
			## Translate backward
			if key == 'S':
				self.vel_msg.linear.x = -round(self.speed_value,2)
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				print('		Last Command Send: Backward')
				
			## Translate to left
			if key == 'A':
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = round(self.speed_value,2)
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				print('		Last Command Send: Left')
				
			## Translate to right
			if key == 'D':
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = -round(self.speed_value,2)
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				print('		Last Command Send: Right')
				
			## Rotate counter clockwise
			if key == 'Q':
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = round(self.speed_value,2)
				self.pub_cmd_vel.publish(self.vel_msg)
				print('		Last Command Send: Rotate (Left)')
				
			## Rotate clockwise
			if key == 'E':
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = -round(self.speed_value,2)
				self.pub_cmd_vel.publish(self.vel_msg)
				print('		Last Command Send: Rotate (Right)')
				
			## Stop
			if key == 'H':
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				print('		Last Command Send: Stop')
				
			if key == 'X':
				self.OvR_msg= 5
				self.pub_override.publish(self.OvR_msg)
				print('		Last Command Send: Autonomous')
				
			if key == 'C':
				self.OvR_msg = 10
				self.pub_override.publish(self.OvR_msg)
				print('		Last Command Send: CANCEL CONTROLLER')

			if self.VALUE_CENTRO>1.5:
				key='W'
				rospy.sleep(0.5)
			else:
				key='D'
				rospy.sleep(1)
			print("distance  ",self.VALUE_CENTRO)
			
			self.vel_msg.linear.x = 0.0
			self.vel_msg.linear.y = 0.0
			self.vel_msg.linear.z = 0.0
			self.vel_msg.angular.z = 0.0
			self.pub_cmd_vel.publish(self.vel_msg)
			rospy.sleep(5)
			


def main():
	rospy.init_node('keyboard', anonymous = True)
	# ~ print("init Node keyboard")
	keyboard()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()
