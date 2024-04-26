#!/usr/bin/env python3
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
import numpy as np

import os
from std_msgs.msg import Int8

class image_viewer:
	def __init__(self):
		self.bridge = CvBridge()
		self.rgbImg = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback, queue_size=1, buff_size=2**24)
		#self.Ovr = rospy.Subscriber('/keyboard/override', Int8, self.flag)
		
		self.dataset_path = rospy.get_param("dataset_path")
		
		self.images_path = self.dataset_path + '/images/'
		self.xml_path = self.dataset_path + '/xml/'

		print('========== From Launch file ========== ')
		print("Dataset Path:", self.dataset_path)
		
		self.override = 0
		self.cont =   0
		self.index =   0
		
		if not os.path.exists(self.images_path): os.mkdir(self.images_path)
		if not os.path.exists(self.xml_path): os.makedirs(self.xml_path)
		
	def callback(self,data):
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		scale = cv2.resize(frame, (320, 240))
		
		if(self.override == 1):
			self.cont+= 1
			
			if (self.cont % 10 ==0):
				self.index += 1
				img_name= 'asus_objeto_5_' + str(self.index) + '.jpg'
				output_file = self.images_path + img_name
				cv2.imwrite(output_file,scale)
				
				print("	img: ", img_name)
		
		cv2.imshow('Image Viewer - RGB', scale)
		cv2.waitKey(1)

	def flag(self,msg):
		if msg.data == 5:
			self.override = 1
		else:
			self.override = 0
		#~ print ("Override: ", msg.data)
		
def main():
	rospy.init_node('Dataset_Generation', anonymous = True)
	print("init Node Image Viewer")
	image_viewer()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()
		sys.exit()

if __name__ == '__main__':
	main()
