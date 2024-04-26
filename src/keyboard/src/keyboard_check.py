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
from nav_msgs.msg import Odometry
from math import sqrt
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
		
		print(self.init_msg)
		print('		Speed:', round(self.speed_value,2))
		print(self.final_msg)
		self.sub_odometry = rospy.Subscriber('/bebop2/odometry_sensor1/odometry', Odometry, self.odometry_callback)
		self.cmd_vel_timeout = rospy.Duration(5)  # Timeout duration in seconds
		self.cmd_vel_sent_time = rospy.Time.now()
		self.command_executed = False
		self.commanded_vel=Twist()
		self.cmd_vel()
	def odometry_callback(self, msg):
        # Check if the drone's velocity matches the commanded velocity
		actual_linear_velocity = sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2 + msg.twist.twist.linear.z ** 2)
		commanded_linear_velocity = sqrt(self.commanded_vel.linear.x ** 2 + self.commanded_vel.linear.y ** 2 + self.commanded_vel.linear.z ** 2)
		if actual_linear_velocity >= commanded_linear_velocity:
			self.command_executed = True
	def cmd_vel(self):
		rate = rospy.Rate(10)  # 10 Hz
		while not rospy.is_shutdown():
			key = getkey()
			
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
			## Ascend
			if key == keys.UP:
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = round(self.speed_value,2)
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				self.cmd_vel_sent_time = rospy.Time.now()
				self.commanded_vel = self.vel_msg
				self.command_executed = False
				print(self.init_msg)
				print('		Speed:', round(self.speed_value,2))
				print('		Last Command Send: Ascend')
				print(self.final_msg)
			## Descend
			if key == keys.DOWN:
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = -round(self.speed_value,2)
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				self.cmd_vel_sent_time = rospy.Time.now()
				self.commanded_vel = self.vel_msg
				self.command_executed = False
				print(self.init_msg)
				print('		Speed:', round(self.speed_value,2))
				print('		Last Command Send: Descend')
				print(self.final_msg)
			## Translate forward 
			if key == keys.W:
				self.vel_msg.linear.x = round(self.speed_value,2)
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				self.cmd_vel_sent_time = rospy.Time.now()
				self.commanded_vel = self.vel_msg
				self.command_executed = False
				print(self.init_msg)
				print('		Speed:', round(self.speed_value,2))
				print('		Last Command Send: Forward')
				print(self.final_msg)
			## Translate backward
			if key == keys.S:
				self.vel_msg.linear.x = -round(self.speed_value,2)
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				self.cmd_vel_sent_time = rospy.Time.now()
				self.commanded_vel = self.vel_msg
				self.command_executed = False
				print(self.init_msg)
				print('		Speed:', round(self.speed_value,2))
				print('		Last Command Send: Backward')
				print(self.final_msg)
			## Translate to left
			if key == keys.A:
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = round(self.speed_value,2)
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				self.cmd_vel_sent_time = rospy.Time.now()
				self.commanded_vel = self.vel_msg
				self.command_executed = False
				print(self.init_msg)
				print('		Speed:', round(self.speed_value,2))
				print('		Last Command Send: Left')
				print(self.final_msg)
			## Translate to right
			if key == keys.D:
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = -round(self.speed_value,2)
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				self.cmd_vel_sent_time = rospy.Time.now()
				self.commanded_vel = self.vel_msg
				self.command_executed = False
				print(self.init_msg)
				print('		Speed:', round(self.speed_value,2))
				print('		Last Command Send: Right')
				print(self.final_msg)
			## Rotate counter clockwise
			if key == keys.Q:
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = round(self.speed_value,2)
				self.pub_cmd_vel.publish(self.vel_msg)
				self.cmd_vel_sent_time = rospy.Time.now()
				self.commanded_vel = self.vel_msg
				self.command_executed = False
				print(self.init_msg)
				print('		Speed:', round(self.speed_value,2))
				print('		Last Command Send: Rotate (Left)')
				print(self.final_msg)
			## Rotate clockwise
			if key == keys.E:
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = -round(self.speed_value,2)
				self.pub_cmd_vel.publish(self.vel_msg)
				self.cmd_vel_sent_time = rospy.Time.now()
				self.commanded_vel = self.vel_msg
				self.command_executed = False
				print(self.init_msg)
				print('		Speed:', round(self.speed_value,2))
				print('		Last Command Send: Rotate (Right)')
				print(self.final_msg)
			## Stop
			if key == keys.H:
				self.vel_msg.linear.x = 0.0
				self.vel_msg.linear.y = 0.0
				self.vel_msg.linear.z = 0.0
				self.vel_msg.angular.z = 0.0
				self.pub_cmd_vel.publish(self.vel_msg)
				self.cmd_vel_sent_time = rospy.Time.now()
				self.commanded_vel = self.vel_msg
				self.command_executed = False
				print(self.init_msg)
				print('		Speed:', round(self.speed_value,2))
				print('		Last Command Send: Stop')
				print(self.final_msg)
			start_time = rospy.Time.now()
			while not self.command_executed and rospy.Time.now() - start_time < self.cmd_vel_timeout:
				rate.sleep()
			if self.command_executed:
				rospy.loginfo("Command executed successfully!")
			else:
				rospy.loginfo("Command execution failed or timed out.")
			rate.sleep()



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
