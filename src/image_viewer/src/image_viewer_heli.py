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
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

class image_viewer:
	def __init__(self):
		self.bridge = CvBridge()
		self.rgbImg = rospy.Subscriber('/camera/rgb', Image, self.callback, queue_size=1, buff_size=2**24)
		self.depthImg = rospy.Subscriber('/camera/depth', Image, self.depth_image, queue_size=1, buff_size=2**24)
		self.topic_pose = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)
		self.topic_velocity=rospy.Subscriber('/bebop2/cmd_vel', Twist, self.cmd_vel_callback)

		#------------

		self.pub_objects_distances = rospy.Publisher('/camera/objects_distances', Float32MultiArray, queue_size=10)

		self.pub_goal = rospy.Publisher('/camera/goal', Int8, queue_size=10)
		self.pub_goal_dist = rospy.Publisher('/camera/dist_goal', Float64, queue_size=10)

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
		self.model_path = '/home/nilda/IntRob2023/src/image_viewer/src/yolo/best_heli.pt'
		self.model = YOLO(self.model_path)  # load a custom model
		self.threshold = 0.7
		self.class_name_dict = {0: 'goal'}#,1: 'chair', 2: 'sofa', 3: 'fridge', 4: 'barbell', 5: 'bed', 6: 'tv', 7: 'ball', 8: 'cabinet', 9: 'glass', 10: 'portrait', 11: 'door', 12: 'shelf', 13: 'wall'}
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
		self.x_points=[[0,0],[260,0],[0,220],[260,220],[150,150]]
		self.y_points=[[260,220],[520,220],[260,440],[520,440],[350,220]]

		self.reseach_goal=0
		self.reseach_goal_dist=0.0
		self.altitude=0.0
		self.robot_name = 'bebop2'
		self.angle_goal=0

	def pose_callback(self,data):
		model_states = data
		model_names = model_states.name
		index = model_names.index(self.robot_name)
		
		self.x = model_states.pose[index].position.x
		self.y = model_states.pose[index].position.y
		self.z = model_states.pose[index].position.z
		
		self.qw = model_states.pose[index].orientation.x
		self.qx = model_states.pose[index].orientation.y
		self.qy = model_states.pose[index].orientation.z
		self.qz = model_states.pose[index].orientation.w

		(self.roll, self.pitch, self.yaw) = euler_from_quaternion([self.qw, self.qx, self.qy, self.qz])

	def cmd_vel_callback(self,data):
		# This callback function will be called whenever a message is received on the /bebop2/cmd_vel topic
		# Process the received Twist message here
		self.linear_x = data.linear.x
		self.linear_y = data.linear.y
		self.linear_z = data.linear.z
		self.angular_x = data.angular.x
		self.angular_y = data.angular.y
		self.angular_z = data.angular.z

	def calculate_length(self,endpoint1, endpoint2):
		return math.sqrt((endpoint2[0] - endpoint1[0])**2 + (endpoint2[1] - endpoint1[1])**2)
    
	# Calculate the angles between the lines using the law of cosines
	def calculate_angle(self, side1, side2, opposite_side):
		return math.degrees(math.acos((side1**2 + side2**2 - opposite_side**2) / (2 * side1 * side2)))

	# Calculate the signed angle between two vectors
	def calculate_signed_angle(self, vector1, vector2):
		cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]
		dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]
		return math.degrees(math.atan2(cross_product, dot_product))

	def callback(self,data):
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		scale = cv2.resize(frame, (520, 440))
		distancias_objetos=[]
		'''
		distancias_objetos=[]
		for i in range(len(self.x_points)):
			print(i,self.x_points[i][0],self.y_points[i][0],self.x_points[i][1],self.y_points[i][1])
			distance_= self.get_distance(self.x_points[i][0],self.y_points[i][0],self.x_points[i][1],self.y_points[i][1])
			#distance_= self.get_distance(int(x1),int(y1),int(x2),int(y2))
			#distance_= self.get_distance(int(self.x_points[i]),int(self.y_points[i]),int(self.x_points[i+1]),int(self.y_points[i+1]))
			distancias_objetos.append(distance_)
		'''
		#print("shape",self.depth_array.shape)
		distancias_objetos.append(self.get_distance(150,150,350,220))#1 centro

		distancias_objetos.append(self.get_distance(0,0,260,220)) #2 top left

		distancias_objetos.append(self.get_distance(260,0,520,220))#3 top right

		distancias_objetos.append(self.get_distance(0,220,260,440))#4 bottom left

		distancias_objetos.append(self.get_distance(260,220,520,440))#5 bottom right


		#para dibujar las lineas 
		scale=cv2.circle(scale, (260, 440), 20, (255, 0, 0), -1) #robot
		scale=cv2.circle(scale, (260, 220), 3, (255, 0, 0), -1) #centro
		scale=cv2.line(scale,(260,440),(260,220),(255,0,0),4) #linea de robot a centro
		
		msg = Float32MultiArray()
		msg.data = distancias_objetos
		self.pub_objects_distances.publish(msg)

		results = self.model(scale,verbose=False)[0]
		for result in results.boxes.data.tolist():
			x1, y1, x2, y2, score, class_id = result
			cx = int((x1 + x2)/2)
			cy=int((y1+y2)/2)
			if score > self.threshold:
				cv2.rectangle(scale, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
				cv2.circle(scale, (cx, cy), 3, (0, 255, 0), -1)
				cv2.putText(scale, self.class_name_dict[int(class_id)].upper(), (int(x1), int(y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
				#print("detect an object")
				if class_id==0: #encontro ball
					#print("encontro goal")
					self.reseach_goal_dist= self.get_distance(int(x1),int(y1),int(x2),int(y2))
					self.reseach_goal=1
					scale=cv2.circle(scale, (cx, cy), 3, (255, 0, 0), -1) #objetivo

					#print("research goal",self.reseach_goal,self.reseach_goal_dist)
					
					scale=cv2.line(scale,(260,440),(cx,cy),(255,0,0),4) #linea de robot a objetivo
					scale=cv2.line(scale,(cx,cy),(260,220),(255,0,0),4) #linea de objetivo a centro

					line1_endpoints = ((260, 440), (260, 220)) 
					line2_endpoints = ((260, 440), (cx, cy))
					line3_endpoints = ((cx, cy), (260, 220))

					# Calculate the vectors representing the line segments
					vector1 = (line1_endpoints[1][0] - line1_endpoints[0][0], line1_endpoints[1][1] - line1_endpoints[0][1])
					vector2 = (line2_endpoints[1][0] - line2_endpoints[0][0], line2_endpoints[1][1] - line2_endpoints[0][1])
					vector3 = (line3_endpoints[1][0] - line3_endpoints[0][0], line3_endpoints[1][1] - line3_endpoints[0][1])

					# Calculate the signed angles between the lines
					angle1 = self.calculate_signed_angle(vector2, vector1)
					angle2 = self.calculate_signed_angle(vector3, vector2)
					angle3 = self.calculate_signed_angle(vector1, vector3)

					self.angle_goal=angle1



		try:
			ros_image_msg = self.bridge.cv2_to_imgmsg(scale, "bgr8")
			self.image_pub.publish(ros_image_msg)
		except CvBridgeError as e:
			rospy.logerr("Error converting OpenCV image to ROS image: {}".format(e))
		if self.reseach_goal==1:
			self.pub_goal.publish(self.reseach_goal)
			self.pub_goal_dist.publish(self.reseach_goal_dist)		



		print("Parte del estado, distancia de las 5 secciones",distancias_objetos)
		print("Si ha encontrado el objetivo",self.reseach_goal, " distancia ",self.reseach_goal_dist, " angulo",self.angle_goal )
		print("altitud",self.z)
		print("velocidad lineal",self.linear_x,self.linear_y,self.linear_z,"velocidad angular",self.angular_x,self.angular_y,self.angular_z)
		print("roll, pitch, yaw",self.roll,self.pitch,self.yaw)

	def depth_image(self,msg_depth):
		bridge = CvBridge()
		try:
			depth_image = bridge.imgmsg_to_cv2(msg_depth, desired_encoding="passthrough")
			depth_array = np.array(depth_image, dtype=np.float32)
			depth_array=cv2.resize(depth_array, (520, 440))#cv2.resize(depth_array,)
			self.depth_array=depth_array
			#grayscale_image = cv2.cvtColor(self.depth_array, cv2.COLOR_BGR2GRAY)
			#cv2.imwrite("/home/nilda/profundidad.jpg",grayscale_image)
		except CvBridgeError:
			print("error")
	def get_distance(self,x1,y1,x2,y2):
		num_points = 20
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
			return 0.5#float('nan')

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
