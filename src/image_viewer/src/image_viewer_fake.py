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
import tensorflow.compat.v1 as tf
import models
class image_viewer:
	def __init__(self):
		self.bridge = CvBridge()
		self.rgbImg = rospy.Subscriber('/camera/rgb', Image, self.callback, queue_size=1, buff_size=2**24)
		self.depthImg = rospy.Subscriber('/camera/depth', Image, self.depth_image, queue_size=1, buff_size=2**24)
		#------------

		self.pub_objects_distances = rospy.Publisher('/camera/objects_distances', Float32MultiArray, queue_size=10)

		self.pub_goal = rospy.Publisher('/camera/goal', Int8, queue_size=10)
		self.pub_goal_dist = rospy.Publisher('/camera/dist_goal', Float64, queue_size=10)

		self.depth_array = np.array([])


		self.depth_array_fake = np.array([])
		self.frame_array = np.array([])

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
		self.x_points=[[0,0],[260,0],[0,220],[260,220],[150,150]]
		self.y_points=[[260,220],[520,220],[260,440],[520,440],[350,220]]

		self.reseach_goal=0
		self.reseach_goal_dist=0.0
	
	def callback(self,data):
		frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		scale = cv2.resize(frame, (520, 440))
		self.frame_array=scale
		self.predict()
		self.publicar_()
		'''
		distancias_objetos=[]
		distancias_objetos.append(self.get_distance(150,150,350,220))#1 centro

		distancias_objetos.append(self.get_distance(0,0,260,220)) #2 top left

		distancias_objetos.append(self.get_distance(260,0,520,220))#3 top right

		distancias_objetos.append(self.get_distance(0,220,260,440))#4 bottom left

		distancias_objetos.append(self.get_distance(260,220,520,440))#5 bottom right

		
		print("objetos",distancias_objetos)
		msg = Float32MultiArray()
		msg.data = distancias_objetos
		#msg.dist = distancias_objetos
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
				print("detect an object")
				if class_id==7: #encontro ball
					print("encontro goal")
					self.reseach_goal_dist= self.get_distance(int(x1),int(y1),int(x2),int(y2))
					self.reseach_goal=1
					print("research goal",self.reseach_goal,self.reseach_goal_dist)

		try:
			ros_image_msg = self.bridge.cv2_to_imgmsg(scale, "bgr8")
			self.image_pub.publish(ros_image_msg)
		except CvBridgeError as e:
			rospy.logerr("Error converting OpenCV image to ROS image: {}".format(e))
		if self.reseach_goal==1:
			self.pub_goal.publish(self.reseach_goal)
			self.pub_goal_dist.publish(self.reseach_goal_dist)		
		'''
	def publicar_(self):
		distancias_objetos=[]
		distancias_objetos.append(self.get_distance_fake(150,150,350,220))#1 centro

		distancias_objetos.append(self.get_distance_fake(0,0,260,220)) #2 top left

		distancias_objetos.append(self.get_distance_fake(260,0,520,220))#3 top right

		distancias_objetos.append(self.get_distance_fake(0,220,260,440))#4 bottom left

		distancias_objetos.append(self.get_distance_fake(260,220,520,440))#5 bottom right

		
		print("objetos",distancias_objetos)
		msg = Float32MultiArray()
		msg.data = distancias_objetos
		#msg.dist = distancias_objetos
		self.pub_objects_distances.publish(msg)

		results = self.model(self.frame_array,verbose=False)[0]
		for result in results.boxes.data.tolist():
			x1, y1, x2, y2, score, class_id = result
			cx = int((x1 + x2)/2)
			cy=int((y1+y2)/2)
			if score > self.threshold:
				cv2.rectangle(self.frame_array, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
				cv2.circle(self.frame_array, (cx, cy), 3, (0, 255, 0), -1)
				cv2.putText(self.frame_array, self.class_name_dict[int(class_id)].upper(), (int(x1), int(y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
				print("detect an object")
				if class_id==7: #encontro ball
					print("encontro goal")
					self.reseach_goal_dist= self.get_distance_fake(int(x1),int(y1),int(x2),int(y2))
					self.reseach_goal=1
					print("research goal",self.reseach_goal,self.reseach_goal_dist)

		try:
			ros_image_msg = self.bridge.cv2_to_imgmsg(self.frame_array, "bgr8")
			self.image_pub.publish(ros_image_msg)
		except CvBridgeError as e:
			rospy.logerr("Error converting OpenCV image to ROS image: {}".format(e))
		if self.reseach_goal==1:
			self.pub_goal.publish(self.reseach_goal)
			self.pub_goal_dist.publish(self.reseach_goal_dist)	


	def depth_image(self,msg_depth):
		bridge = CvBridge()
		try:
			depth_image = bridge.imgmsg_to_cv2(msg_depth, desired_encoding="passthrough")
			depth_array = np.array(depth_image, dtype=np.float32)
			depth_array=cv2.resize(depth_array, (520, 440))#cv2.resize(depth_array,)
			self.depth_array=depth_array
		except CvBridgeError:
			print("error")
	def predict(self):
		print("predict")
		tf.reset_default_graph()
		# Default input size
		height = 228
		width = 304
		channels = 3
		batch_size = 1
		img=self.frame_array
		img=cv2.resize(img, (width, height))
		#img = img.resize([width,height])
		img = np.array(img).astype('float32')
		img = np.expand_dims(np.asarray(img), axis = 0)
	
		# Create a placeholder for the input image
		
		tf.disable_v2_behavior()
		#x = tf.placeholder(shape=[None, 2], dtype=tf.float32)
		input_node = tf.placeholder(tf.float32, shape=(None, height, width, channels))
		model_data_path="/home/nilda/IntRob2023/src/image_viewer/src/NYU_FCRN-checkpoint/NYU_FCRN.ckpt"
		# Construct the network
		net = models.ResNet50UpProj({'data': input_node}, batch_size, 1, False)
		with tf.Session() as sess:
			# Load the converted parameters
			print('Loading the model')

			# Use to load from ckpt file
			saver = tf.train.Saver()     
			saver.restore(sess, model_data_path)

			# Use to load from npy file
			#net.load(model_data_path, sess) 

			# Evalute the network for the given image
			pred = sess.run(net.get_output(), feed_dict={input_node: img})
			try:
				self.depth_array_fake=cv2.resize(pred[0,:,:,0], (520, 440))
			except:
				print("error")
		sess.close() 

        
        

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
	
	def get_distance_fake(self,x1,y1,x2,y2):
		num_points = 20
		random_points = []
		for _ in range(num_points):
			random_x = random.uniform(x1, x2)
			random_y = random.uniform(y1, y2)
			random_points.append(self.depth_array_fake[int(random_y), int(random_x)])
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
