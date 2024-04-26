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
import time
import os
from std_msgs.msg import Int8
import pybboxes as pbx

class image_viewer:
	def __init__(self):
		self.override = 2
		self.cont =   0
		self.index =   147
		self.cont2 =   0
		self.index2 =   1
		self.bridge = CvBridge()
		self.rgbImg = rospy.Subscriber('/bebop2/front_camera/rgb/image_raw', Image, self.callback, queue_size=1, buff_size=2**24)
		#self.rgbImgDepth=rospy.Subscriber("/bebop2/front_camera/depth/image_raw", Image,self.convert_depth_image, queue_size=1)
		
		self.dataset_path = '/home/nilda/IntRob2023/src/image_viewer/src/dataset/'
		
		self.pub_override = rospy.Publisher('/image/override', Int8, queue_size=10)
		

		print('========== From Launch file  ========== ')
		print("Dataset Path:", self.dataset_path)
		
		
		
		
		
	def callback(self,data):

		
		
		self.cont+= 1
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		scale = cv2.resize(frame, (520, 440))
		if (self.cont % 50 ==0):
			if(self.cont==1):
				print("				Initializing data capture...")
			
			self.index += 1
			
					
			img_name= 'img_' + str(self.index) + '.jpg'
			
			output_file = self.dataset_path + img_name
			cv2.imwrite(output_file,scale)
			
			
			print("	img: ", img_name)
		cv2.imshow('Image Viewer - RGB', scale)
					
			
	def callback2(self,data):

		
		
		self.cont2+= 1
		
		if (self.cont2 % 50 ==0):
			if(self.cont2==1):
				print("				Initializing data capture...")
				
				
			depth_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
			depth_image = cv2.resize(depth_image, (520, 440))
			# Convert the depth image to a Numpy array since most cv2 functions
			# require Numpy arrays.
			depth_array = np.array(depth_image, dtype=np.float32)
			# Normalize the depth image to fall between 0 (black) and 1 (white)
			cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
			# At this point you can display the result properly:
			# cv2.imshow('Depth Image', depth_display_image)
			# If you write it as it si, the result will be a image with only 0 to 1 values.
			# To actually store in a this a image like the one we are showing its needed
			# to reescale the otuput to 255 gray scale.
			#cv2.imwrite('capture_depth.png',frame*255)
			
			
			#frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
			#scale = cv2.resize(frame, (520, 440))
			self.index2 += 1
			
					
			img_name= 'depth_' + str(self.index2) + '.jpg'
			
			output_file = self.dataset_path2 + img_name
			cv2.imwrite(output_file,depth_array*255)
			
			print("	img: ", img_name)
				
		
	def convert_depth_image(self,ros_image):
		#rospy.sleep(1)
		bridge = CvBridge()
		# Use cv_bridge() to convert the ROS image to OpenCV format
		try:
		#Convert the depth image using the default passthrough encoding
			depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
			depth_array = np.array(depth_image, dtype=np.float32)
			depth_array=cv2.resize(depth_array, (520, 440))#cv2.resize(depth_array,)
			self.depth_array=depth_array
		except CvBridgeError:
			print("error")
		self.cont2+= 1
		
		if (self.cont2 % 50 ==0):
			if(self.cont2==1):
				print("				Initializing data capture...")
			
			img_name= 'depth_' + str(self.index2) + '.jpg'
			self.index2 += 1
			output_file = self.dataset_path + img_name
			cv2.imwrite(output_file,depth_array)
		
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
