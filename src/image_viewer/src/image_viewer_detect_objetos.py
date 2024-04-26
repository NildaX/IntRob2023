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
from time import time
from ultralytics import YOLO
from std_msgs.msg import Int8
from std_msgs.msg import Float64
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import random

class image_viewer:
	def __init__(self):
		self.bridge = CvBridge()
		self.rgbImg = rospy.Subscriber('/camera/rgb', Image, self.callback, queue_size=1, buff_size=2**24)
		self.depthImg = rospy.Subscriber('/camera/depth', Image, self.depth_image, queue_size=1, buff_size=2**24)
		#------------
		self.pub_section = rospy.Publisher('/camera/section', Int8, queue_size=10)
		self.pub_distance = rospy.Publisher('/camera/distance', Float64, queue_size=10)

		self.pub_objects = rospy.Publisher('/camera/objects', Int32MultiArray, queue_size=10)
		self.pub_objects_distances = rospy.Publisher('/camera/objects_distances', Float32MultiArray, queue_size=10)
		self.pub_objects_secciones = rospy.Publisher('/camera/objects_sections', Int32MultiArray, queue_size=10)

		self.pub_distance_centro = rospy.Publisher('/camera/distance_centro', Float64, queue_size=10)
		self.depth_array = np.array([])

		self.image_pub = rospy.Publisher('/camera/rgb/modified', Image, queue_size=10)
        

		##------------
		
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
		self.model_path = '/home/nilda/Turtle/src/turtlebot3_codigos/modelos/best.pt'
		self.model = YOLO(self.model_path)  # load a custom model
		self.threshold = 0.5
		self.class_name_dict = {0: 'table',1: 'chair', 2: 'sofa', 3: 'fridge', 4: 'barbell', 5: 'bed', 6: 'tv', 7: 'ball', 8: 'cabinet', 9: 'glass', 10: 'portrait', 11: 'door', 12: 'shelf', 13: 'wall'}
		self.bandera_identificacion=0
		self.cont =   0
		self.index =   0
		self.section=0
		self.number_of_object=6
		self.section=''
		self.section_num=0
		self.count_image=0
		self.count_image_real=0
		self.name_actual='mouse'
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
		scale = cv2.resize(frame, (520, 440))
		results = self.model(scale,verbose=False)[0]
		cx = 0.0
		cy = 0.0
		center_x1=130
		center_x2=390
		center_y1=110
		center_y2=330
		dimensions=(520,440)
		aux=results.boxes.data.tolist()
		print(len(aux))
		resultados=list(range(len(aux)))
		distancias=list(range(len(aux)))
		i=0
		self.bandera_identificacion=0
		objetos_reconocidos=[]
		distancias_objetos=[]
		secciones_objetos=[]
		for result in results.boxes.data.tolist():
			x1, y1, x2, y2, score, class_id = result

			cx = int((x1 + x2)/ 2)
			cy = int((y1 + y2)/ 2)

			if score > self.threshold:
				self.bandera_identificacion=1
				if self.name_actual==self.class_name_dict[int(class_id)]:
					self.count_image+=1
				cv2.rectangle(scale, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
				cv2.circle(scale, (cx, cy), 3, (0, 255, 0), -1)
				cv2.putText(scale, self.class_name_dict[int(class_id)].upper(), (cx, cy),cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
				if (center_x1 <= cx < center_x2) and (center_y1 <= cy < center_y2):
					self.section='Center'
					self.section_num=1
				elif cx>=0 and cy>=0 and cx<=dimensions[1]/2 and cy<=dimensions[0]/2:
					self.section='Top left'
					self.section_num=2
				elif cx>=0 and cy>=0 and cx>dimensions[1]/2 and cy<dimensions[0]/2:
					self.section='Top rigth'
					self.section_num=3
				elif cx>=0 and cy>=0 and cx<dimensions[1]/2 and cy>dimensions[0]/2:
					self.section='Bottom left'
					self.section_num=4
				elif cx>=0 and cy>=0 and cx>dimensions[1]/2 and cy>dimensions[0]/2:
					self.section='Bottom right'
					self.section_num=5
				distance_= self.get_distance(int(x1),int(y1),int(x2),int(y2))
				print("The object "+self.class_name_dict[int(class_id)]+" is "+self.section+" with a distance of  "+str(distance_)+" m")
				resultados[i]=self.section_num
				distancias[i]=distance_
				objetos_reconocidos.append(int(class_id))
				distancias_objetos.append(distance_)
				secciones_objetos.append(self.section_num)
				
			else:
				self.section_num=0
				resultados[i]=0
				distancias[i]=999
			i+=1

		
		##------------distancia del centro
		distance_centro= self.get_distance(int(150),int(150),int(350),int(220))
		self.pub_distance_centro.publish(distance_centro)
		print("distancia del centro ", distance_centro)
		##----------



		self.count_image_real+=1
		distancias,resultados=self.burbuja(distancias,resultados)
		if len(resultados)>0 and self.bandera_identificacion==1:
			self.pub_section.publish(resultados[0])
			self.pub_distance.publish(distancias[0])

			msg = Int32MultiArray()
			msg.data = objetos_reconocidos
			#msg.dist = distancias_objetos
			self.pub_objects.publish(msg)

			msg = Float32MultiArray()
			msg.data = distancias_objetos
			#msg.dist = distancias_objetos
			self.pub_objects_distances.publish(msg)

			msg = Int32MultiArray()
			msg.data = secciones_objetos
			#msg.dist = distancias_objetos
			self.pub_objects_secciones.publish(msg)

		else:
			cy=260
			cx=220
			###------------si no hay objetos debo de revisar si hay algo enfrente
			distance_= self.get_distance(int(150),int(150),int(350),int(220))
			print("no hay objetos, ver el centro",distance_)
			self.pub_section.publish(0)
			self.pub_distance.publish(distance_)
			cv2.circle(scale, (cx, cy), 3, (0, 255, 0), -1)

		scale = cv2.rectangle(
				img = scale,
				pt1 = (150, 150),
				pt2 = (350, 220), 
				color = (0, 0, 255)
			)
		try:
			ros_image_msg = self.bridge.cv2_to_imgmsg(scale, "bgr8")
			self.image_pub.publish(ros_image_msg)
		except CvBridgeError as e:
			rospy.logerr("Error converting OpenCV image to ROS image: {}".format(e))
		#except:
  		#	print("An exception occurred") 
		#cv2.imshow('Image Viewer - RGB', scale)
		#cv2.imwrite(self.images_path +'detection.jpg',scale)
		#cv2.waitKey(1)
		
		#cv2.imshow('Image Viewer - RGB', scale)
		#cv2.imshow("Image Viewer - Depth", self.depthimg)
		#cv2.waitKey(1)

	def depth_image(self,msg_depth):
		bridge = CvBridge()
		try:
			depth_image = bridge.imgmsg_to_cv2(msg_depth, desired_encoding="passthrough")
			depth_array = np.array(depth_image, dtype=np.float32)
			depth_array=cv2.resize(depth_array, (520, 440))#cv2.resize(depth_array,)
			self.depth_array=depth_array
		except CvBridgeError:
			print("error")
	def get_distance(self,x1,y1,x2,y2):
		num_points = 10
		random_points = []
		for _ in range(num_points):
			random_x = random.uniform(x1, x2)
			random_y = random.uniform(y1, y2)
			random_points.append(self.depth_array[int(random_y), int(random_x)])
		random_points = [x for x in random_points if not np.isnan(x)]
		#print("random_points",random_points)
		if len(random_points)>0:
			return np.amin(random_points)
		else:
			return 999#float('nan')

	def burbuja(self,arreglo,arreglo1):
    		n = len(arreglo)
    		for i in range(n-1):
    			for j in range(n-1-i):
    				if arreglo[j] > arreglo[j+1]:
    					arreglo[j], arreglo[j+1] = arreglo[j+1], arreglo[j]
    					arreglo1[j], arreglo1[j+1] = arreglo1[j+1], arreglo1[j]
    		return arreglo,arreglo1
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
