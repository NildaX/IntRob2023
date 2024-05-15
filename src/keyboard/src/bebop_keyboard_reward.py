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
class keyboard:
	def __init__(self):
		self.pub_takeoff = rospy.Publisher('/bebop2/takeoff', Empty, queue_size=10)
		self.pub_land = rospy.Publisher('/bebop2/land', Empty, queue_size=10)
		self.pub_cmd_vel = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=10)
		self.pub_override = rospy.Publisher('/keyboard/override', Int8, queue_size=10)
		self.array_states_discrete=[]
		self.array_states=[999,999,999,999,0,0,999,999,999,999,999,999,999,999,999]
		self.objetcs_detected_distances = rospy.Subscriber('/camera/state_discrete', Float32MultiArray, self.flag5)
		self.objetcs_detected = rospy.Subscriber('/camera/state', Float32MultiArray, self.flag6)
		self.vel_msg = Twist()
		self.OvR_msg = 0
		self.speed_value = 0.3
		self.cumulated_reward=0
		self.cumulated_steps=0
		self.archivo=pd.DataFrame(columns=["action","reward"])

		
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
		self.array_states_discrete=msg.data
	def flag6(self,msg):
		self.array_states=msg.data

	def _compute_reward(self):
		print("discrete", self.array_states)
		visibility_reward=100*self.array_states[5]
		if self.array_states[5]==1:
		    distance_reward=-self.array_states[6]*10 #distance_to_goal
		    angle_reward=-(abs(self.array_states[7]))*10 #angle_to_goal
		else:
		    distance_reward=-100 #distance_to_goal
		    angle_reward=-100 #angle_to_goal
		distances_reward=(self.array_states[0]+self.array_states[1]+self.array_states[2])*5
		print(distance_reward,angle_reward,visibility_reward,distances_reward)
		reward = round(distance_reward + angle_reward + visibility_reward+distances_reward,2)
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
				key = getkey()
				if key == keys.RIGHT:
					self.speed_value +=0.05
					if self.speed_value >= 1.0:
						self.speed_value = 1.0
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Increment speed')
					print(self.final_msg)
					reward=self._compute_reward()
					new_row = {"action": "RIGHT", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
				if key == keys.LEFT:
					self.speed_value -=0.05
					if self.speed_value <= 0.0:
						self.speed_value = 0.0
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Decrement speed')
					print(self.final_msg)
					reward=self._compute_reward()
					new_row = {"action": "LEFT", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
				## Takeoff
				if key == keys.T:
					msg = Empty()
					self.pub_takeoff.publish(msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Takeoff')
					print(self.final_msg)
				## Land
				if key == keys.SPACE:
					msg = Empty()
					self.pub_land.publish(msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Land')
					print(self.final_msg)
					reward=self._compute_reward()
					new_row = {"action": "Land", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
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
					reward=self._compute_reward()
					new_row = {"action": "AsceRIGHTnd", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
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
					reward=self._compute_reward()
					new_row = {"action": "Descend", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
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
					reward=self._compute_reward()
					new_row = {"action": "forward", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
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
					reward=self._compute_reward()
					new_row = {"action": "backward", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
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
					reward=self._compute_reward()
					new_row = {"action": "Left", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
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
					reward=self._compute_reward()
					new_row = {"action": "Right", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
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
					reward=self._compute_reward()
					new_row = {"action": "Rotate RIGHT", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
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
					reward=self._compute_reward()
					new_row = {"action": " Rotate left", "reward": reward}
					self.archivo.loc[len(self.archivo)] = new_row
				## Stop
				if key == keys.H:
					self.vel_msg.linear.x = 0.0
					self.vel_msg.linear.y = 0.0
					self.vel_msg.linear.z = 0.0
					self.vel_msg.angular.z = 0.0
					self.pub_cmd_vel.publish(self.vel_msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: Stop')
					print(self.final_msg)
					
				if key == keys.X:
					print('		Last Command Send: Termino')
					print('FINAL CUMULATED REWARD',self.cumulated_reward)
					bandera=1
				if key == keys.C:
					self.OvR_msg = 10
					self.pub_override.publish(self.OvR_msg)
					print(self.init_msg)
					print('		Speed:', round(self.speed_value,2))
					print('		Last Command Send: CANCEL CONTROLLER')
					print(self.final_msg)
					reward=self._compute_reward()
				rospy.sleep(1)
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)



				if self.array_states[5]==1 and self.array_states[6]<=1.3:
					self.cumulated_reward+=1000
					bandera=1
				if self.array_states[8]<0.15:
                			self.cumulated_reward+=(-1000)
                			bandera=1
			print('		Last Command Send: Termino')
			print('FINAL CUMULATED REWARD',self.cumulated_reward)
			if contador==0:
				new_row = {"action": "Final", "reward": self.cumulated_reward}
				self.archivo.loc[len(self.archivo)] = new_row
				self.archivo.to_csv('rewards.csv', index=False)
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
