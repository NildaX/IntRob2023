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
		self.rgbImg = rospy.Subscriber('/bebop2/front_camera/rgb/image_raw', Image, self.callback, queue_size=1, buff_size=2**24)
		
	def callback(self,data):
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		
		cv2.imshow('Image Viewer - RGB', frame)
		cv2.waitKey(1)

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
