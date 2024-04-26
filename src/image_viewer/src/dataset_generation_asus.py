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
from ultralytics import YOLO
import pybboxes as pbx

class image_viewer:
	def __init__(self):
		self.bridge = CvBridge()
		self.rgbImg = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback, queue_size=1, buff_size=2**24)
		self.Ovr = rospy.Subscriber('/arm/override', Int8, self.flag)
		
		self.dataset_path = rospy.get_param("dataset_path")
		
		self.pub_override = rospy.Publisher('/image/override', Int8, queue_size=10)
		
		self.images_path = self.dataset_path + '/images/'
		self.xml_path = self.dataset_path + '/labels/'
		##----variables for detection
		self.model_path = '/home/nilda/IntRob2023/deep_learning/yolo-proyecto/runs/detect/train-auto/weights/best.pt'

		print('========== From Launch file  ========== ')
		print("Dataset Path:", self.dataset_path)
		print("Dataset Path for model:", self.model_path)
		
		self.override = 0
		self.cont =   0
		self.index =   0
		self.section=0
		self.number_of_object=6
		if not os.path.exists(self.images_path): os.mkdir(self.images_path)
		if not os.path.exists(self.xml_path): os.makedirs(self.xml_path)
		
		# Load a model
		self.model = YOLO(self.model_path)  # load a custom model
		self.threshold = 0.5
		self.class_name_dict = {0: 'cup', 1: 'can', 2: 'box', 3: 'mouse', 4: 'bottle', 5: 'rose'}
		#self.class_name_dict = {0: 'dog', 1: 'person', 2: 'cat', 3: 'tv', 4: 'car', 5: 'meatballs', 6: 'marinara sauce', 7: 'tomato soup', 8: 'chicken noodle soup', 9: 'french onion soup', 10: 'chicken breast', 11: 'ribs', 12: 'pulled pork', 13: 'hamburguer', 14: 'cavity', 15: 'cono'}
		self.count_image=0
		self.count_image_real=0
		self.name_actual='mouse'
		
	def callback(self,data):
		if (self.override == 0):
			frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
			scale = cv2.resize(frame, (520, 440))
			cv2.imshow('Image Viewer - RGB', scale)
			cv2.waitKey(1)
		
		if(self.override == 1 and self.section==0):
			self.find_object(data)
			
		
		if(self.override == 2):
			##----for recognition
			self.cont+= 1
			
			if (self.cont % 10 ==0):
				if(self.cont==1):
					print("				Initializing data capture...")
				frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
				scale = cv2.resize(frame, (520, 440))
				self.index += 1
				##---for save the label in the txt file
				image_copy = np.array(scale)
				dimensions=scale.shape
				gray = cv2.cvtColor(scale, cv2.COLOR_BGR2GRAY)

				canny = cv2.Canny(gray, 10,150)
				canny = cv2.dilate(canny, None, iterations=1)
				canny = cv2.erode(canny, None, iterations=1)
				cnts,_ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


				#----Looking for the bigger contour
				area_list = []
				for c in cnts:
					area = cv2.contourArea(c)
					area_list.append(area)
				if len(area_list)>0:
					bigger = cnts[area_list.index(max(area_list))]

					#---Draw the bigger contour
					area = cv2.contourArea(bigger)
					x,y,w,h = cv2.boundingRect(bigger)
					
					##--Transfor for YOLO format
					
					
					if (x+w)<=dimensions[0] and (y+h)<=dimensions[1]:
						
						yolo_=x,y,x+w,y+h
						cv2.rectangle(image_copy, (x,y), (x+w, y+h), (0,255,0), 2)
						dimensions=scale.shape
						a=pbx.convert_bbox(yolo_, from_type="voc", to_type="yolo", image_size=(dimensions[0],dimensions[1]))
						
						img_name= 'asus_object_'+str(self.number_of_object)+'_' + str(self.index) + '.jpg'
						file_name= 'asus_object_'+str(self.number_of_object)+'_' + str(self.index) + '.txt'
						output_file = self.images_path + img_name
						cv2.imwrite(output_file,scale)
						##--save image and txt file
						file = open(self.xml_path + file_name, "w")
						file.write(str(self.number_of_object)+" "+str(a[0])+" "+str(a[1])+" "+str(a[2])+" "+str(a[3]))
						file.close()
						
						print("	img: ", img_name)
						print("	label: ", file_name)
				
				cv2.imshow('Image Viewer - RGB', image_copy)
				cv2.waitKey(1)
		if(self.override == 3):
			##---changing the counters
			print("				Capture of images ends...")
			self.override = 0
			self.cont =   0
			self.index =   0
			self.section=0
			self.number_of_object+=1
			self.count_image=0
			self.count_image_real=0
			cv2.destroyAllWindows()
		if(self.override == 4):
			if (self.section==0):
				self.find_object(data)
			else:
				self.recognized_object(data)
		if(self.override == 5):
			self.override = 0
			self.section=0
			cv2.destroyAllWindows()
	
	def flag(self,msg):
		
		if msg.data == 5:
			self.override = 1
			
		elif msg.data == 10:
			self.override = 2
		elif msg.data == 15:
			self.override = 3
		elif msg.data == 20:
			self.override = 4
		elif msg.data == 25:
			self.override = 5
			print(self.count_image,self.count_image_real)
			print("total:",self.count_image/self.count_image_real)
			self.count_image=0
			self.count_image=0
		else:
			self.override = 0
		
	def find_object(self,data):
		print("Taking the first photo...")
		#----for the the first photo
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		scale = cv2.resize(frame, (520, 440))
		image_copy = np.array(scale)
		gray = cv2.cvtColor(scale, cv2.COLOR_BGR2GRAY)

		canny = cv2.Canny(gray, 10,150)
		canny = cv2.dilate(canny, None, iterations=1)
		canny = cv2.erode(canny, None, iterations=1)
		cnts,_ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


		#----Looking for the bigger contour
		area_list = []
		for c in cnts:
		    area = cv2.contourArea(c)
		    area_list.append(area)

		bigger = cnts[area_list.index(max(area_list))]

		#---Draw the bigger contour
		area = cv2.contourArea(bigger)
		x,y,w,h = cv2.boundingRect(bigger)
		cv2.rectangle(image_copy, (x,y), (x+w, y+h), (0,255,0), 2)
		dimensions=scale.shape
		if x>=0 and y>=0 and x<=dimensions[1]/2 and y<=dimensions[0]/2:
			self.section=3
		elif x>=0 and y>=0 and x>dimensions[1]/2 and y<dimensions[0]/2:
			self.section=4
		elif x>=0 and y>=0 and x<dimensions[1]/2 and y>dimensions[0]/2:
			self.section=1
		elif x>=0 and y>=0 and x>dimensions[1]/2 and y>dimensions[0]/2:
			self.section=2
		#self.section=2
		
		cv2.imwrite(self.images_path +'first_image_object_rect_' + str(self.number_of_object) + '.jpg',image_copy)
		
		self.pub_override.publish(self.section)
		time.sleep(10)
	def recognized_object(self,data):
		print("				Recognizing the object")
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		scale = cv2.resize(frame, (520, 440))
		results = self.model(scale)[0]
		cx = 0.0
		cy = 0.0

		for result in results.boxes.data.tolist():
		    x1, y1, x2, y2, score, class_id = result

		    cx = int((x1 + x2)/ 2)
		    cy = int((y1 + y2)/ 2)

		    if score > self.threshold:
		    	if self.name_actual==self.class_name_dict[int(class_id)]:
		    		self.count_image+=1
		    	cv2.rectangle(scale, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
		    	cv2.circle(scale, (cx, cy), 3, (0, 255, 0), -1)
		    	cv2.putText(scale, self.class_name_dict[int(class_id)].upper(), (cx, cy),
				        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
				
		self.count_image_real+=1
		cv2.imshow('Image Viewer - RGB', scale)
		#cv2.imwrite(self.images_path +'detection.jpg',scale)
		cv2.waitKey(1)
				
		
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
