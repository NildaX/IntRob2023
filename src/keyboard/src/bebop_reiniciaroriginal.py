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
from gazebo_msgs.msg import ModelStates,ModelState
from geometry_msgs.msg import Pose
import time
import random
import pandas as pd
class keyboard:
	def __init__(self):
		self.pub_takeoff = rospy.Publisher('/bebop2/takeoff', Empty, queue_size=10)
		self.pub_land = rospy.Publisher('/bebop2/land', Empty, queue_size=10)
		self.pub_cmd_vel = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=10)
		self.pub_override = rospy.Publisher('/keyboard/override', Int8, queue_size=10)
		self.sub_pose=rospy.Subscriber("/gazebo/model_states", ModelStates, self.pose_callback)
		self.reset_pose_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
		#self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
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
		
		self.array_states=[]


		self.objetcs_detected_distances = rospy.Subscriber('/camera/state', Float32MultiArray, self.flag5)
		

		self.class_name_dict = {0: 'table', 1: 'chair', 2: 'sofa', 3: 'fridge', 4: 'barbell', 5: 'bed', 6: 'tv', 7: 'ball', 8: 'cabinet', 9: 'glass', 10: 'portrait', 11: 'door', 12: 'shelf', 13: 'wall'}
		self.class_name_sections = {0: 'CENTER',1:'TOP LEFT',2:'TOP RIGHT',3:'BOTTOM LEFT',4:'BOTTOM RIGHT'}
		self.class_name_actions = {'a': 'LEFT','w':'FORWARD','s':'STOP','d':'RIGHT','x':'REVERSE'}
		
		#self.keys_possibles=['UP','DOWN','W','S','A','D','Q','E','H']#['SPACE','UP','DOWN','W','S','A','D','Q','E','H']
		self.keys_possibles=['UP','W','S','A','D','Q','E']#['SPACE','UP','DOWN','W','S','A','D','Q','E','H']
		print(self.keys_possibles)
		self.new_data = { 'num_episodio':[0],'seccion_0': [0],'seccion_1': [0],'seccion_2': [0],'seccion_3': [0],'seccion_4': [0], 'rearch_goal': [0],'distance_goal': [0],'angle_goal': [0],'altitude': [0], 'linear_x': [0],'linear_y':[0],'linear_z':[0],'roll':[0],'pitch':[0],'yaw':[0],'action':['W']}
		self.df = pd.DataFrame(self.new_data)
		self.bandera=0
		self.pose_received = False
		self.initial_pose=[]
		self.cmd_vel()
		
	def flag5(self,msg):
		#print("flag5")
		self.array_states=msg.data
		#if(self.ara)

	def pose_callback(self, data):
		if not self.pose_received:
		    try:
		        idx = data.name.index('bebop2')
		        self.initial_pose = data.pose[idx]
		        self.pose_received = True
		        rospy.loginfo("Initial Pose Saved:\n{}".format(self.initial_pose))
		    except ValueError:
		        rospy.logwarn("Robot not found in ModelStates message.")


	def cmd_vel(self):
		rospy.sleep(5)
		key='T'
		contador_guardar=0
		contador_reiniciar=0
		num_episodio=0
		MAX_EPISODES=10#10
		MAX_ACCIONES=20#20
		while not rospy.is_shutdown():
			while num_episodio<MAX_EPISODES:
				if contador_reiniciar==MAX_ACCIONES and self.pose_received:
					print("contador reiniciar 20")

					print('----stop----')
					self.vel_msg = Twist()
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = 0.0
					self.speed_value = 0.3
					self.pub_cmd_vel.publish(self.vel_msg)
					rospy.sleep(0.5)
					#---INITIAL POSE
					print("---initial pose")
					reset_pose_msg = ModelState()
					reset_pose_msg.model_name = 'bebop2'
					reset_pose_msg.pose = self.initial_pose
					self.reset_pose_publisher.publish(reset_pose_msg)
					rospy.sleep(0.5)

					print("----land---")
					msg = Empty()
					self.pub_land.publish(msg)
					rospy.sleep(0.5)



					print("-----------Robot pose reset to initial pose-------")
					contador_reiniciar=0
					num_episodio+=1
					key='T'
					print("------------Episode: ",num_episodio,"-------------")
				else:
					contador_reiniciar+=1
				key_anterior=key
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

				##------SELECCIONAR ALEATORIAMENTE UNA OPCION
				if(len(self.array_states)>2):
					new_data = { 'num_episodio':num_episodio,'seccion_0': self.array_states[0],'seccion_1': self.array_states[1],'seccion_2':self.array_states[2],'seccion_3': self.array_states[3],'seccion_4': self.array_states[4], 'rearch_goal': self.array_states[5],'distance_goal': self.array_states[6],'angle_goal': self.array_states[7],'altitude': self.array_states[8], 'linear_x': self.vel_msg.linear.x,'linear_y':self.vel_msg.linear.y,'linear_z':self.vel_msg.linear.z,'roll':self.array_states[12],'pitch':self.array_states[13],'yaw':self.array_states[14],'action':key_anterior}
					new_df = pd.DataFrame(new_data, index=[0])
					# Concatenate existing DataFrame with new_df
					self.df = pd.concat([self.df, new_df], ignore_index=True)
				rospy.sleep(1)
				key=random.choice(self.keys_possibles)
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				rospy.sleep(0.5)
				#contador_guardar+=1
				#if contador_guardar%10==0:
				
				#if contador_guardar>200:
				excel_file_path = '/home/nilda/Documentos/archivos/data.xlsx'
				(self.df).to_excel(excel_file_path, index=False)
			print("finalizo episodios en:",num_episodio)

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
