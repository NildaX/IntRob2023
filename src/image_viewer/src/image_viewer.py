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
import numpy as np

class image_viewer:
	def __init__(self):
		self.bridge = CvBridge()
		self.rgbImg = rospy.Subscriber('/camera/rgb', Image, self.callback, queue_size=1, buff_size=2**24)
		self.depthImg = rospy.Subscriber('/camera/depth', Image, self.depth_image, queue_size=1, buff_size=2**24)
		
		self.dist = rospy.get_param("dist")
		self.number_float = rospy.get_param("number_float")
		self.undistort = rospy.get_param("undistort")
		self.camW = rospy.get_param("Camera.width")
		self.CamH = rospy.get_param("Camera.height")
		self.image_width = rospy.get_param("image_width")
		self.image_height = rospy.get_param("image_height")
		self.camera_matrix = rospy.get_param("/camera_matrix/data")
		self.distortion_coefficients = rospy.get_param("/distortion_coefficients/data")
		self.projection_matrix = rospy.get_param("/projection_matrix/data")

		print('========== From Launch file ========== ')
		print("dist", self.dist)
		print('========== From config.yaml file ========== ')
		print("number_float", self.number_float)
		print("undistort", self.undistort)
		print('========== From bebop_cam.yaml file ========== ')
		print("camW", self.camW)
		print("CamH", self.CamH)
		print('========== From bebop_calib.yaml file ========== ')
		print("image_width", self.image_width)
		print("image_height", self.image_height)
		print("camera_matrix", self.camera_matrix)
		print("distortion_coefficients", self.distortion_coefficients)
		print("projection_matrix", self.projection_matrix)
		
		self.matrix = np.zeros((3, 3))
		self.pmatrix = np.zeros((3, 3))
		self.dist = np.zeros((1, 5))
		
	def load_params(self,camera_matrix, distortion_coefficients, projection_matrix):
		for i in range(9):
				self.matrix[i // 3][i % 3] = float(self.camera_matrix[i])
				self.pmatrix[i // 3][i % 3] = float(self.projection_matrix[i])
		for i in range(5):
				self.dist[0][i]= float(self.distortion_coefficients[i])
		# ~ print ('matrix:', self.matrix)
		# ~ print ('pmatrix:', self.pmatrix)
		# ~ print ('dist:', self.dist)
		
	def callback(self,data):
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		
		if(self.undistort):
			self.load_params(self.camera_matrix, self.distortion_coefficients, self.projection_matrix)
			frame_undis = cv2.undistort(frame, self.matrix, self.dist, self.pmatrix)
			cv2.imshow('Image Viewer - RGB undistort', frame_undis)
		
		cv2.imshow('Image Viewer - RGB', frame)
		cv2.imshow("Image Viewer - Depth", self.depthimg)
		cv2.waitKey(1)

	def depth_image(self,msg_depth):
		
		# The depth image is a single-channel float32 image
		# the values is the distance in mm in z axis
		cv_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")
		# Convert the depth image to a Numpy array since most cv2 functions
		# require Numpy arrays.
		cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
		# Normalize the depth image to fall between 0 (black) and 1 (white)
		self.depthimg = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
		# Resize to the desired size

def main():
	rospy.init_node('image_viewer', anonymous = True)
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
