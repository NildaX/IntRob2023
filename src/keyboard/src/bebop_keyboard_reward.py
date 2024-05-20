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
from std_msgs.msg import Float32MultiArray
import pandas as pd
import numpy as np
class keyboard:
	def __init__(self):
		self.pub_takeoff = rospy.Publisher('/bebop2/takeoff', Empty, queue_size=10)
		self.pub_land = rospy.Publisher('/bebop2/land', Empty, queue_size=10)
		self.pub_cmd_vel = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=10)
		self.pub_override = rospy.Publisher('/keyboard/override', Int8, queue_size=10)
		self.array_states_discrete=[0,0,0,0,0,0,0,0,0]
		self.array_states=[999,999,999,999,0,0,999,999,999,999,999,999,999,999,999]
		self.array_states_discrete_p=[0,0,0,0,0,0,0,0,0]
		self.array_states_p=[999,999,999,999,0,0,999,999,999,999,999,999,999,999,999] #the p is about previous
		self.objetcs_detected_distances = rospy.Subscriber('/camera/state_discrete', Float32MultiArray, self.flag5)
		self.objetcs_detected = rospy.Subscriber('/camera/state', Float32MultiArray, self.flag6)
		self.vel_msg = Twist()
		self.OvR_msg = 0
		self.speed_value = 0.3
		self.cumulated_reward=0
		self.cumulated_steps=0
		self.archivo=pd.DataFrame(columns=['section_0','section_1','section_2','section_3','section_4','see_goal','distance_goal','angle_goal','altitude','section_0_n','section_1_n','section_2_n','section_3_n','section_4_n','see_goal_n','distance_goal_n','angle_goal_n','altitude_n',"action","reward_acumulated","reward"])
		self.archivo_discreto = pd.DataFrame(columns=['section_0','section_1','section_2','section_3','section_4','see_goal','distance_goal','angle_goal','altitude','section_0_n','section_1_n','section_2_n','section_3_n','section_4_n','see_goal_n','distance_goal_n','angle_goal_n','altitude_n',"action","reward_acumulated","reward"])
		
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
		
		print(self.init_msg)
		print('		Speed:', round(self.speed_value,2))
		print(self.final_msg)
		self.cmd_vel()
	def flag5(self,msg):
		self.array_states_discrete=np.round(msg.data, 2) 
		
	def flag6(self,msg):
		self.array_states=np.round(msg.data, 2) 

	def _compute_reward(self):
		print("discrete", self.array_states)
		visibility_reward=100*self.array_states[5]
		if self.array_states[5]==1:
		    distance_reward=-self.array_states[6] #distance_to_goal
		    angle_reward=-(abs(self.array_states[7])) #angle_to_goal
		    print("distancia cuando ve",distance_reward,angle_reward)
		else:
		    distance_reward=-9 #distance_to_goal
		    angle_reward=0 #angle_to_goal
		    print("distancia cuando no ve",distance_reward,angle_reward)
		print(distance_reward,angle_reward,visibility_reward)
		reward = round(distance_reward + angle_reward + visibility_reward,2)
		rospy.logdebug("reward=" + str(reward))
		self.cumulated_reward += reward
		rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
		self.cumulated_steps += 1
		rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
		return reward

	def cmd_vel(self):
		bandera=0
		contador=0
		while not rospy.is_shutdown():
			while(bandera==0):
				previous_state=self.array_states
				previous_dstate=self.array_states_discrete
				key = getkey()
				## Takeoff
				if key == keys.T:
					msg = Empty()
					self.pub_takeoff.publish(msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Takeoff')
					print(self.final_msg)
				## Ascend
				if key == keys.UP:
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = round(self.speed_value,2)
					self.vel_msg.angular.z = 0.0
					self.pub_cmd_vel.publish(self.vel_msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Ascend')
					print(self.final_msg)
					action="Ascend"
				## Descend
				if key == keys.DOWN:
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = -round(self.speed_value,2)
					self.vel_msg.angular.z = 0.0
					self.pub_cmd_vel.publish(self.vel_msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Descend')
					print(self.final_msg)
					action="Descend"
				## Translate forward 
				if key == keys.W:
					self.vel_msg.linear.x = round(self.speed_value,2)
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = 0.0
					self.pub_cmd_vel.publish(self.vel_msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Forward')
					print(self.final_msg)
					action="forward"
				## Translate backward
				if key == keys.S:
					self.vel_msg.linear.x = -round(self.speed_value,2)
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = 0.0
					self.pub_cmd_vel.publish(self.vel_msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Backward')
					print(self.final_msg)
					action="backward"
				## Translate to left
				if key == keys.A:
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = round(self.speed_value,2)
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = 0.0
					self.pub_cmd_vel.publish(self.vel_msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Left')
					print(self.final_msg)
					action="Left"
				## Translate to right
				if key == keys.D:
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = -round(self.speed_value,2)
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = 0.0
					self.pub_cmd_vel.publish(self.vel_msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Right')
					print(self.final_msg)
					action="Right"
				## Rotate counter clockwise
				if key == keys.Q:
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = round(self.speed_value,2)
					self.pub_cmd_vel.publish(self.vel_msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Rotate (Left)')
					print(self.final_msg)
					action="Rotate right"
				## Rotate clockwise
				if key == keys.E:
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = -round(self.speed_value,2)
					self.pub_cmd_vel.publish(self.vel_msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Rotate (Right)')
					print(self.final_msg)
					action="Rotate left"
				if key == keys.X:
					print('		Last Command Send: Termino')
					print('FINAL CUMULATED REWARD',self.cumulated_reward)
					bandera=1
				
				rospy.sleep(1)

				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				if key!=keys.T:
					reward=self._compute_reward()
					new_row = {'section_0': previous_state[0],'section_1': previous_state[1],'section_2': previous_state[2],'section_3': previous_state[3],'section_4': previous_state[4], 'see_goal': previous_state[5],'distance_goal':previous_state[6],'angle_goal':previous_state[7],'altitude':previous_state[8],'section_0_n':self.array_states [0],'section_1_n':self.array_states [1],'section_2_n':self.array_states [2],'section_3_n':self.array_states [3],'section_4_n':self.array_states [4],'see_goal_n':self.array_states [5],'distance_goal_n':self.array_states [6],'angle_goal_n':self.array_states [7],'altitude_n':self.array_states [8],"action": action, "reward_acumulated": self.cumulated_reward,"reward": reward}
					new_row_d = {'section_0': previous_dstate[0],'section_1': previous_dstate[1],'section_2': previous_dstate[2],'section_3': previous_dstate[3],'section_4': previous_dstate[4], 'see_goal': previous_dstate[5],'distance_goal':previous_dstate[6],'angle_goal':previous_dstate[7],'altitude':previous_dstate[8],'section_0_n':self.array_states_discrete [0],'section_1_n':self.array_states_discrete [1],'section_2_n':self.array_states_discrete [2],'section_3_n':self.array_states_discrete [3],'section_4_n':self.array_states_discrete [4],'see_goal_n':self.array_states_discrete [5],'distance_goal_n':self.array_states_discrete [6],'angle_goal_n':self.array_states_discrete [7],'altitude_n':self.array_states_discrete [8],"action": action, "reward_acumulated": self.cumulated_reward,"reward":reward}

					self.archivo.loc[len(self.archivo)] = new_row
					self.archivo_discreto.loc[len(self.archivo_discreto)] = new_row_d


					if self.array_states[5]==1 and self.array_states[6]<=1.3:
						self.cumulated_reward+=1000
						bandera=1
					if self.array_states[8]<0.15:
								self.cumulated_reward+=(-1000)
								bandera=1
			print('		Last Command Send: Termino')
			print('FINAL CUMULATED REWARD',self.cumulated_reward)
			if contador==0:
				#new_row = {"action": "Final", "reward": self.cumulated_reward}
				#self.archivo.loc[len(self.archivo)] = new_row
				self.archivo.to_csv('rewards.csv', index=False)
				self.archivo_discreto.to_csv('rewards_discreto.csv', index=False)
				contador=1


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
