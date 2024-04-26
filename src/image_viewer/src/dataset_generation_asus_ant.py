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

		print('========== From Launch file ========== ')
		print("Dataset Path:", self.dataset_path)
		
		self.override = 0
		self.cont =   0
		self.index =   0
		self.section=0
		self.number_of_object=5
		if not os.path.exists(self.images_path): os.mkdir(self.images_path)
		if not os.path.exists(self.xml_path): os.makedirs(self.xml_path)
		##----variables for detection
		self.model_path = '/home/nilda/IntRob2023/deep_learning/yolo-proyecto/runs/detect/train/weights/best.pt'

		# Load a model
		self.model = YOLO(self.model_path)  # load a custom model
		self.threshold = 0.5
		self.class_name_dict = {0: 'cup', 1: 'can', 2: 'box', 3: 'mouse', 4: 'bottle', 5: 'rose'}
		#self.class_name_dict = {0: 'dog', 1: 'person', 2: 'cat', 3: 'tv', 4: 'car', 5: 'meatballs', 6: 'marinara sauce', 7: 'tomato soup', 8: 'chicken noodle soup', 9: 'french onion soup', 10: 'chicken breast', 11: 'ribs', 12: 'pulled pork', 13: 'hamburguer', 14: 'cavity', 15: 'cono'}
		
		
	def callback(self,data):
		
		
		if(self.override == 1 and self.section==0):
			self.find_object(data)
			
		
		if(self.override == 2):
			##para reconocer
			self.cont+= 1
			
			if (self.cont % 10 ==0):
				print("initializing data capture...")
				frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
				scale = cv2.resize(frame, (320, 240))
				
				self.index += 1
				img_name= 'asus_object_'+str(self.number_of_object)+'_' + str(self.index) + '.jpg'
				output_file = self.images_path + img_name
				cv2.imwrite(output_file,scale)
				
				print("	img: ", img_name)
		
				cv2.imshow('Image Viewer - RGB', scale)
				cv2.waitKey(1)
		if(self.override == 3):
			##seria poner contadores en 0 tal vez
			print("capture of images ends...")
			self.override = 0
			self.cont =   0
			self.index =   0
			self.section=0
			self.number_of_object+=1
			cv2.destroyAllWindows()
		if(self.override == 4):
			if (self.section==0):
				print("inicializatinf object recognition")
				self.find_object(data)
			else:
				self.recognized_object(data)
		if(self.override == 5):
			self.override = 0
			self.section=0
			cv2.destroyAllWindows()
	def prueba(self):
		if(self.override == 1):
			print("Taking the first photo...")
			self.pub_override.publish(3)
		if(self.override == 2):
			print("initializing data capture...")
			
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
		else:
			self.override = 0
		#self.prueba()
		#~ print ("Override: ", msg.data)
	def find_object(self,data):
		print("Taking the first photo...")
		#----for the the first photo
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		scale = cv2.resize(frame, (320, 240))
		image_copy = np.array(scale)
		gray = cv2.cvtColor(scale, cv2.COLOR_BGR2GRAY)

		canny = cv2.Canny(gray, 10,150)
		canny = cv2.dilate(canny, None, iterations=1)
		canny = cv2.erode(canny, None, iterations=1)
		cnts,_ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


		#Buscas el contorno más grande
		lista_areas = []
		for c in cnts:
		    area = cv2.contourArea(c)
		    lista_areas.append(area)


		#Te quedas con el area más grande
		mas_grande = cnts[lista_areas.index(max(lista_areas))]

		#Representas el contorno más grande
		area = cv2.contourArea(mas_grande)
		x,y,w,h = cv2.boundingRect(mas_grande)
		cv2.rectangle(image_copy, (x,y), (x+w, y+h), (0,255,0), 2)
		dimensions=scale.shape
		if x>=0 and y>=0 and x<=dimensions[1]/2 and y<=dimensions[0]/2:
			print("esta en seccion 3")
			self.section=3
		elif x>=0 and y>=0 and x>dimensions[1]/2 and y<dimensions[0]/2:
			print("esta en seccion 4")
			self.section=4
		elif x>=0 and y>=0 and x<dimensions[1]/2 and y>dimensions[0]/2:
			print("esta en seccion 1")
			self.section=1
		elif x>=0 and y>=0 and x>dimensions[1]/2 and y>dimensions[0]/2:
			print("esta en seccion 2")
			self.section=2
		
		cv2.imwrite(self.images_path +'first_image_object_rect_' + str(self.number_of_object) + '.jpg',image_copy)
		#cv2.imshow('Image Viewer - RGB', image_copy)
		#cv2.waitKey(1)
		self.pub_override.publish(self.section)
		time.sleep(10)
	def recognized_object(self,data):
		print("reconociendo objecto")
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		scale = cv2.resize(frame, (320, 240))
		#time1 = time()
		results = self.model(scale)[0]
		#print("results", results)

		#trans = time() - time1
		#print("\nTime: ", trans)

		cx = 0.0
		cy = 0.0

		for result in results.boxes.data.tolist():
		    x1, y1, x2, y2, score, class_id = result

		    cx = int((x1 + x2)/ 2)
		    cy = int((y1 + y2)/ 2)

		    if score > self.threshold:
		    	cv2.rectangle(scale, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
		    	cv2.circle(scale, (cx, cy), 3, (0, 255, 0), -1)
		    	cv2.putText(scale, self.class_name_dict[int(class_id)].upper(), (int(x1), int(y1 - 10)),
				        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)

		#fps = 'FPS: {:.1f}'.format(1/trans)
		#print(fps)
		cv2.imshow('Test', scale)
		cv2.imwrite(self.images_path +'detecttion.jpg',scale)
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
