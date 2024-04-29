import rospy
import time
from openai_ros_b import robot_gazebo_env
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from openai_ros_b.openai_ros_common import ROSLauncher
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from keras.models import load_model
import message_filters
import cv2
from std_msgs.msg import Int8
from gazebo_msgs.msg import ModelStates,ModelState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import random
from ultralytics import YOLO
from tf.transformations import euler_from_quaternion
import math
from math import sqrt
class BebopEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """

    def __init__(self, ros_ws_abspath):
        """
        Initializes a new TurtleBot3Env environment.
        TurtleBot3 doesnt use controller_manager, therefore we wont reset the
        controllers in the standard fashion. For the moment we wont reset them.

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.

        The Sensors: The sensors accesible are the ones considered usefull for AI learning.

        Sensor Topic List:
        * /odom : Odometry readings of the Base of the Robot
        * /imu: Inertial Mesuring Unit that gives relative accelerations and orientations.
        * /scan: Laser Readings

        Actuators Topic List: /cmd_vel,

        Args:
        """
        rospy.logdebug("Start BebopEnv INIT...")
        # Variables that we give through the constructor.
        # None in this case


        # We launch the ROSlaunch that spawns the robot into the world
        #ROSLauncher(rospackage_name="turtlebot3_gazebo",
        #            launch_file_name="put_robot_in_world.launch",
        #            ros_ws_abspath=ros_ws_abspath)
        ROSLauncher(rospackage_name="rotors_gazebo",
                    launch_file_name="put_bebop_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)
        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = ["imu"]

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(BebopEnv, self).__init__(controllers_list=self.controllers_list,
                                            robot_name_space=self.robot_name_space,
                                            reset_controls=False,
                                            start_init_physics_parameters=False)

        self.gazebo.unpauseSim()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        #rospy.Subscriber("/odom", Odometry, self._odom_callback)
        rospy.Subscriber("/bebop2/odometry_sensor1/odometry", Odometry, self._odom_callback)
        rospy.Subscriber("/imu", Imu, self._imu_callback)
        rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)

        #self._cmd_vel_pub = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop2/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/bebop2/land', Empty, queue_size=10)
        self._cmd_vel_pub = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=10)
        self.pub_override = rospy.Publisher('/keyboard/override', Int8, queue_size=10)
        self.pub_reset = rospy.Publisher('/bebop2/reset', Empty, queue_size=10)
        self.vel_msg = Twist()
        self.OvR_msg = 0
        self.speed_value = 0.3
        self.band_imagen=0
        ##----para verificar
        self.cmd_vel_timeout = rospy.Duration(20)
        self.cmd_vel_sent_time = rospy.Time.now()
        self.command_executed = False
        self.commanded_vel=Twist()


        self._check_publishers_connection()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished BebopEnv INIT...")
        
        
        #--------Agregados
        self.bridge = CvBridge()
		# Exportar el modelo a 'SavedModel'
        self.reset_pose_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        self.subscriber_image=rospy.Subscriber("/bebop2/front_camera/rgb/image_raw", Image, self._image_rgb_callback)
        self.subscriber_image_depth=rospy.Subscriber("/bebop2/front_camera/depth/image_raw", Image, self.depth_image_callback, queue_size=1, buff_size=2**24)
        topic_pose = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)
        topic_velocity=rospy.Subscriber('/bebop2/cmd_vel', Twist, self.cmd_vel_callback)


        self.pub_state = rospy.Publisher('/camera/state', Float32MultiArray, queue_size=10)
        self.image_pub = rospy.Publisher('/camera/rgb/modified', Image, queue_size=10)


        self.repre_state=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        self.repre_state_discrete=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        self.model_path = '/home/nilda/IntRob2023/src/image_viewer/src/yolo/best_heli_sim.pt'#best_heli.pt'
        self.model = YOLO(self.model_path)  # load a custom model

        self.threshold = 0.7
        self.class_name_dict = {0: 'goal'}
        self.COLLISION_BAND=0
        self.rate_ver = rospy.Rate(60)
        self.bandera_pose_ok=0
        '''
        self.SAVE=1
        self.rgbImg = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.rgbImgDepth = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.ts = message_filters.TimeSynchronizer([self.rgbImg, self.rgbImgDepth], 20)
        self.ts.registerCallback(self.callback)
        self.pub_save = rospy.Publisher('/turtlebot/save', Int8, queue_size=10)
        '''

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        self._check_odom_ready()
        #self._check_laser_scan_ready()
        self.subscriber_image=rospy.Subscriber("/bebop2/front_camera/rgb/image_raw", Image, self._image_rgb_callback)
        self.subscriber_image_depth=rospy.Subscriber("/bebop2/front_camera/depth/image_raw", Image, self.depth_image_callback, queue_size=1, buff_size=2**24)
        self.check_image_raw_ready()
        self.check_image_depth_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        self.odom = None
        rospy.logdebug("Waiting for /odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/bebop2/odometry_sensor1/odometry", Odometry, timeout=5.0)
                rospy.logdebug("Current /odom READY=>")

            except:
                rospy.logerr("Current /odom not ready yet, retrying for getting odom in turtlebot3_env")

        return self.odom

    def _check_imu_ready(self):
        self.imu = None
        rospy.logdebug("Waiting for /imu to be READY...")
        while self.imu is None and not rospy.is_shutdown():
            try:
                self.imu = rospy.wait_for_message("/imu", Imu, timeout=5.0)
                rospy.logdebug("Current /imu READY=>")

            except:
                rospy.logerr("Current /imu not ready yet, retrying for getting imu")

        return self.imu

    def _check_laser_scan_ready(self):
        self.laser_scan = None
        rospy.logdebug("Waiting for /scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
            try:
                self.laser_scan = rospy.wait_for_message("/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /scan READY=>")

            except:
                rospy.logerr("Current /scan not ready yet, retrying for getting laser_scan")
        return self.laser_scan
    def check_image_raw_ready(self):
        self.image_raw = None
        rospy.logdebug("Waiting for /image to be READY...")
        while self.image_raw is None and not rospy.is_shutdown():
            try:
                self.image_raw = rospy.wait_for_message("/bebop2/front_camera/rgb/image_raw", Image, timeout=1.0)
                rospy.logdebug("Current /image READY=>")

            except:
                rospy.logerr("Current /image not ready yet, retrying for getting image")
        print("Current /image ready")
        return self.image_raw
    
    def check_image_depth_ready(self):
        self.image_depth = None
        rospy.logdebug("Waiting for /image_depth to be READY...")
        while self.image_depth is None and not rospy.is_shutdown():
            try:
                self.image_depth = rospy.wait_for_message("/bebop2/front_camera/depth/image_raw", Image, timeout=1.0)
                rospy.logdebug("Current /image_depth READY=>")

            except:
                rospy.logerr("Current /image_depth not ready yet, retrying for getting image_depth")
        print("Current /image depth ready")
        return self.image_depth


    def pose_callback(self,data):
        try:
            model_states = data
            model_names = model_states.name
            index = model_names.index('bebop2')
            self.repre_state[8] = round(model_states.pose[index].position.z,2)
            (roll, pitch, yaw) = euler_from_quaternion([model_states.pose[index].orientation.x,model_states.pose[index].orientation.y,model_states.pose[index].orientation.z, model_states.pose[index].orientation.w])
            self.repre_state[12]=round(roll,2)
            self.repre_state[13]=round(pitch,2)
            self.repre_state[14]=round(yaw,2)
            self.bandera_pose_ok=1
        except:
            #rospy.logerr("Waiting fot pose callback")
            self.bandera_pose_ok=0

    def cmd_vel_callback(self,data):
        self.repre_state[9] = round(data.linear.x,2)
        self.repre_state[10] = round(data.linear.y,2)
        self.repre_state[11] = round(data.linear.z,2)

    def _odom_callback(self, data):
        self.odom = data
        actual_linear_velocity = sqrt(data.twist.twist.linear.x ** 2 + data.twist.twist.linear.y ** 2 + data.twist.twist.linear.z ** 2)
        commanded_linear_velocity = sqrt(self.commanded_vel.linear.x ** 2 + self.commanded_vel.linear.y ** 2 + self.commanded_vel.linear.z ** 2)
        if actual_linear_velocity >= commanded_linear_velocity:
            self.command_executed = True

    def _imu_callback(self, data):
        self.imu = data

    def _laser_scan_callback(self, data):
        self.laser_scan = data
    # Calculate the signed angle between two vectors
    def calculate_signed_angle(self, vector1, vector2):
        cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]
        dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]
        return math.degrees(math.atan2(cross_product, dot_product))

    def _image_rgb_callback(self, data):
        #print("img")
        self.band_imagen=1
        frame_1=self.bridge.imgmsg_to_cv2(data, "bgr8")
        frame_1 = cv2.resize(frame_1, (84, 84))
        self.image_raw_nueva = frame_1
        '''
        scale = cv2.resize(frame_1, (520, 440))
        self.repre_state[0]=round(self.get_distance(150,150,350,220),2)#1 centro
        self.repre_state[1]=round(self.get_distance(0,0,260,220),2) #2 top left
        self.repre_state[2]=round(self.get_distance(260,0,520,220),2)#3 top right
        self.repre_state[3]=round(self.get_distance(0,220,260,440),2)#4 bottom left
        self.repre_state[4]=round(self.get_distance(260,220,520,440),2)#5 bottom right
        #print(self.repre_state)
        scale=cv2.circle(scale, (260, 440), 20, (255, 0, 0), -1) #robot
        scale=cv2.circle(scale, (260, 220), 3, (255, 0, 0), -1) #centro
        scale=cv2.line(scale,(260,440),(260,220),(255,0,0),4) #linea de robot a centro
        results = self.model(scale,verbose=False)[0]
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result
            cx = int((x1 + x2)/2)
            cy=int((y1+y2)/2)
            if score > self.threshold:
                cv2.rectangle(scale, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                cv2.circle(scale, (cx, cy), 3, (0, 255, 0), -1)
                cv2.putText(scale, self.class_name_dict[int(class_id)].upper(), (int(x1), int(y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)#print("detect an object")
                if class_id==0: #encontro ball#print("encontro goal")
                    self.repre_state[6]= round(self.get_distance(int(x1),int(y1),int(x2),int(y2)),2)
                    self.repre_state[5]=1
                    rospy.logerr("GOAAAAL ==>" + str(self.repre_state[5]))
                    scale=cv2.circle(scale, (cx, cy), 3, (255, 0, 0), -1) #objetiv
                    scale=cv2.line(scale,(260,440),(cx,cy),(255,0,0),4) #linea de robot a objetivo
                    scale=cv2.line(scale,(cx,cy),(260,220),(255,0,0),4) #linea de objetivo a centro
                    line1_endpoints = ((260, 440), (260, 220)) 
                    line2_endpoints = ((260, 440), (cx, cy))# Calculate the vectors representing the line segments
                    vector1 = (line1_endpoints[1][0] - line1_endpoints[0][0], line1_endpoints[1][1] - line1_endpoints[0][1])
                    vector2 = (line2_endpoints[1][0] - line2_endpoints[0][0], line2_endpoints[1][1] - line2_endpoints[0][1])# Calculate the signed angles between the lines
                    angle1 = self.calculate_signed_angle(vector2, vector1)
                    self.repre_state[7]=round(angle1,2)
        '''
        '''
        if self.repre_state[5]!=1:
            self.repre_state[6]=999
            self.repre_state[7]=999
        try:
            ros_image_msg = self.bridge.cv2_to_imgmsg(scale, "bgr8")
            self.image_pub.publish(ros_image_msg)
        except CvBridgeError as e:
            rospy.logerr("Error converting OpenCV image to ROS image: {}".format(e))
        rospy.sleep(1)
        '''
        
    def change_state(self):
        self.repre_state[5]=0
        
        self.repre_state[0]=round(self.get_distance(150,150,350,220),2)#1 centro
        self.repre_state[1]=round(self.get_distance(0,0,260,220),2) #2 top left
        self.repre_state[2]=round(self.get_distance(260,0,520,220),2)#3 top right
        self.repre_state[3]=round(self.get_distance(0,220,260,440),2)#4 bottom left
        self.repre_state[4]=round(self.get_distance(260,220,520,440),2)#5 bottom right
        #print(self.repre_state)
        
        #scale=cv2.circle(scale, (260, 440), 20, (255, 0, 0), -1) #robot
        #scale=cv2.circle(scale, (260, 220), 3, (255, 0, 0), -1) #centro
        #scale=cv2.line(scale,(260,440),(260,220),(255,0,0),4) #linea de robot a centro
        contador=0
        max_contador=10
        while contador<max_contador and self.repre_state[5]==0:
            scale = cv2.resize(self.image_raw_nueva, (520, 440))
            results = self.model(scale,verbose=False)[0]
            for result in results.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = result
                cx = int((x1 + x2)/2)
                cy=int((y1+y2)/2)
                if score > self.threshold:
                    cv2.rectangle(scale, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                    cv2.circle(scale, (cx, cy), 3, (0, 255, 0), -1)
                    cv2.putText(scale, self.class_name_dict[int(class_id)].upper(), (int(x1), int(y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)#print("detect an object")
                    if class_id==0: #encontro ball#print("encontro goal")
                        self.repre_state[6]= round(self.get_distance(int(x1),int(y1),int(x2),int(y2)),2)
                        self.repre_state[5]=1
                        rospy.logerr("GOAAAAL ==>" + str(self.repre_state[5]))
                        scale=cv2.circle(scale, (cx, cy), 3, (255, 0, 0), -1) #objetiv
                        scale=cv2.line(scale,(260,440),(cx,cy),(255,0,0),4) #linea de robot a objetivo
                        scale=cv2.line(scale,(cx,cy),(260,220),(255,0,0),4) #linea de objetivo a centro
                        line1_endpoints = ((260, 440), (260, 220)) 
                        line2_endpoints = ((260, 440), (cx, cy))# Calculate the vectors representing the line segments
                        vector1 = (line1_endpoints[1][0] - line1_endpoints[0][0], line1_endpoints[1][1] - line1_endpoints[0][1])
                        vector2 = (line2_endpoints[1][0] - line2_endpoints[0][0], line2_endpoints[1][1] - line2_endpoints[0][1])# Calculate the signed angles between the lines
                        angle1 = self.calculate_signed_angle(vector2, vector1)
                        self.repre_state[7]=round(angle1,2)
            contador+=1
        print("contador",contador)
        if self.repre_state[5]!=1:
            self.repre_state[6]=999
            self.repre_state[7]=999
        ##---discrete, disctances, see goal 0-1, angle to goal 0,1,2, altitude 0,1,2
        self.discretized_state(self.repre_state)

        return True
        '''
        try:
            ros_image_msg = self.bridge.cv2_to_imgmsg(scale, "bgr8")
            self.image_pub.publish(ros_image_msg)
        except CvBridgeError as e:
            rospy.logerr("Error converting OpenCV image to ROS image: {}".format(e))
        '''
    def discretized_state(self,array_1):
        #print(self.repre_state_discrete)
        #print(len(self.repre_state_discrete))
        #print(self.repre_state_discrete[0],self.repre_state_discrete[1],self.repre_state_discrete[2],self.repre_state_discrete[3],self.repre_state_discrete[4],self.repre_state_discrete[5])
        array_aux=[0,0,0,0,0,0,0,0,0]
        print(len(array_aux))
        if array_1[0]<=0.5:
            array_aux[0]=0
        else:
            array_aux[0]=1

        if array_1[1]<=0.5:
            array_aux[1]=0
        else:
            array_aux[1]=1

        if array_1[2]<=0.5:
            array_aux[2]=0
        else:
            array_aux[2]=1

        if array_1[3]<=0.5:
            array_aux[3]=0
        else:
            array_aux[3]=1
        
        if array_1[4]<=0.5:
            array_aux[4]=0
        else:
            array_aux[4]=1

        if array_1[6]<=0.5:
            array_aux[6]=0
        else:
            array_aux[6]=1
        #0 close 1 far
        #0 good 1 too left 2 too right
        if array_1[7]<=-15:
            array_aux[7]=2
        else:
            if array_1[7]>=15:
                array_aux[7]=1
            else:
                array_aux[7]=0
        #8 altitud, 0 good 1 close ground 2 far ground
        if array_1[8]<=0.5:
            array_aux[8]=1
        else:
            if array_1[8]>=1.5:
                array_aux[8]=2
            else:
                array_aux[8]=0

        array_aux[5]=array_1[5]
        self.repre_state_discrete=array_aux
        return self.repre_state_discrete
    def depth_image_callback(self,data):
        bridge = CvBridge()
        try:
            depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32)
            depth_array=cv2.resize(depth_array, (520, 440))#cv2.resize(depth_array,)
            self.depth_array=depth_array
            #print("depth_image_callback")
        except CvBridgeError:
            print("error")


    def get_distance(self,x1,y1,x2,y2):
        num_points = 20
        random_points = []
        for _ in range(num_points):
            random_x = random.uniform(x1, x2)
            random_y = random.uniform(y1, y2)
            random_points.append(self.depth_array[int(random_y), int(random_x)])
        random_points = [x for x in random_points if not np.isnan(x)]#print("random_points",random_points)
        if len(random_points)>0:
            return np.amin(random_points)
        else:
            return 0.99#float('nan')
    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to _cmd_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_cmd_vel_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()

    # Methods that the TrainingEnvironment will need.
    # ----------------------------
    def move_base(self, key):#linear_speed, angular_speed, epsilon=0.05, update_rate=10):
        """
        It will move the base based on the linear and angular speeds given.
        It will wait untill those twists are achived reading from the odometry topic.
        :param linear_speed: Speed in the X axis of the robot base frame
        :param angular_speed: Speed of the angular turning of the robot base frame
        :param epsilon: Acceptable difference between the speed asked and the odometry readings
        :param update_rate: Rate at which we check the odometry.
        :return:
        
        cmd_vel_value = Twist()
        cmd_vel_value.linear.x = linear_speed
        cmd_vel_value.angular.z = angular_speed
        rospy.logdebug("TurtleBot3 Base Twist Cmd>>" + str(cmd_vel_value))
        self._check_publishers_connection()
        self._cmd_vel_pub.publish(cmd_vel_value)
        # self.wait_until_twist_achieved(cmd_vel_value,epsilon,update_rate)
        # Weplace a waitof certain amiunt of time, because this twist achived doesnt work properly
        time.sleep(0.2)
        """
        self.COLLISION_BAND=0
        if key == 'RIGHT':
            self.speed_value +=0.05
            if self.speed_value >= 1.0:
                self.speed_value = 1.0
            print('		Last Command Send: Increment speed')
            
        if key == 'LEFT':
            self.speed_value -=0.05
            if self.speed_value <= 0.0:
                self.speed_value = 0.0
            print('		Last Command Send: Decrement speed')
            
        ## Takeoff
        if key == 'T':
            #while self.bandera_pose_ok==0:
            #    rospy.sleep(1)
            anterior=round(self.repre_state[8],2)
            msg = Empty()
            self.pub_takeoff.publish(msg)
            print('		Last Command Send: Takeoff',anterior,round(self.repre_state[8],2) )
            while anterior==round(self.repre_state[8],2):
                #print("repetir")
                anterior=round(self.repre_state[8],2)
                rospy.sleep(1)
                msg = Empty()
                self.pub_takeoff.publish(msg)
                print('		Last Command Send: Takeoff')
            #print('salio',anterior,round(self.repre_state[8],2) )
        ## Land1
        if key == 'SPACE':
            msg = Empty()
            self.pub_land.publish(msg)
            print('		Last Command Send: Land')
            
        ## Ascend
        if key == 'UP':
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = round(self.speed_value,2)
            self.vel_msg.angular.z = 0.0
            self._cmd_vel_pub.publish(self.vel_msg)
            ##--para verificar
            self.cmd_vel_sent_time = rospy.Time.now()
            self.commanded_vel = self.vel_msg
            self.command_executed = False
            #--
            print('		Last Command Send: Ascend')
            
        ## Descend
        if key == 'DOWN':
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = -round(self.speed_value,2)
            self.vel_msg.angular.z = 0.0
            self._cmd_vel_pub.publish(self.vel_msg)
            ##--para verificar
            self.cmd_vel_sent_time = rospy.Time.now()
            self.commanded_vel = self.vel_msg
            self.command_executed = False
            #--
            print('		Last Command Send: Descend')
            
        ## Translate forward 
        if key == 'W':
            self.vel_msg.linear.x = round(self.speed_value,2)
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = 0.0
            self.vel_msg.angular.z = 0.0
            self._cmd_vel_pub.publish(self.vel_msg)
            ##--para verificar
            self.cmd_vel_sent_time = rospy.Time.now()
            self.commanded_vel = self.vel_msg
            self.command_executed = False
            #--
            print('		Last Command Send: Forward')
            
        ## Translate backward
        if key == 'S':
            self.vel_msg.linear.x = -round(self.speed_value,2)
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = 0.0
            self.vel_msg.angular.z = 0.0
            self._cmd_vel_pub.publish(self.vel_msg)
            ##--para verificar
            self.cmd_vel_sent_time = rospy.Time.now()
            self.commanded_vel = self.vel_msg
            self.command_executed = False
            #--
            print('		Last Command Send: Backward')
            
        ## Translate to left
        if key == 'A':
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = round(self.speed_value,2)
            self.vel_msg.linear.z = 0.0
            self.vel_msg.angular.z = 0.0
            self._cmd_vel_pub.publish(self.vel_msg)
            ##--para verificar
            self.cmd_vel_sent_time = rospy.Time.now()
            self.commanded_vel = self.vel_msg
            self.command_executed = False
            #--
            print('		Last Command Send: Left')
            
        ## Translate to right
        if key == 'D':
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = -round(self.speed_value,2)
            self.vel_msg.linear.z = 0.0
            self.vel_msg.angular.z = 0.0
            self._cmd_vel_pub.publish(self.vel_msg)
            ##--para verificar
            self.cmd_vel_sent_time = rospy.Time.now()
            self.commanded_vel = self.vel_msg
            self.command_executed = False
            #--
            print('		Last Command Send: Right')
            
        ## Rotate counter clockwise
        if key == 'Q':
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = 0.0
            self.vel_msg.angular.z = round(self.speed_value,2)
            self._cmd_vel_pub.publish(self.vel_msg)
            ##--para verificar
            self.cmd_vel_sent_time = rospy.Time.now()
            self.commanded_vel = self.vel_msg
            self.command_executed = False
            #--
            print('		Last Command Send: Rotate (Left)')
            
        ## Rotate clockwise
        if key == 'E':
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = 0.0
            self.vel_msg.angular.z = -round(self.speed_value,2)
            self._cmd_vel_pub.publish(self.vel_msg)
            ##--para verificar
            self.cmd_vel_sent_time = rospy.Time.now()
            self.commanded_vel = self.vel_msg
            self.command_executed = False
            #--
            print('		Last Command Send: Rotate (Right)')
            
        ## Stop
        if key == 'H':
            self.vel_msg.linear.x = 0.0
            self.vel_msg.linear.y = 0.0
            self.vel_msg.linear.z = 0.0
            self.vel_msg.angular.z = 0.0
            self._cmd_vel_pub.publish(self.vel_msg)
            print('		Last Command Send: Stop')
            
        if key == 'X':
            msg = Empty()
            self.pub_reset.publish(msg)
            #print(self.init_msg)
            #print('		Speed:', round(self.speed_value,2))
            print('		Last Command Send: Reset')
            #print(self.final_msg)
            
        if key == 'C':
            self.OvR_msg = 10
            self.pub_override.publish(self.OvR_msg)
            print('		Last Command Send: CANCEL CONTROLLER')

        ##------SELECCIONAR ALEATORIAMENTE UNA OPCION
        #print("before sleep")
        #rospy.sleep(1)

        ##-----------
        
        start_time = rospy.Time.now()
        rospy.sleep(1)
        if key!='T':
            while not self.command_executed and rospy.Time.now() - start_time < self.cmd_vel_timeout:
                self.rate_ver.sleep()
            if self.command_executed:
                rospy.loginfo("Command executed successfully!")
            else:
                rospy.logerr("Command execution failed, The drone crashed.")
                self.COLLISION_BAND=1
        
        ##---------


        self.rate_ver.sleep()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.z = 0.0
        self._cmd_vel_pub.publish(self.vel_msg)
        #print("before other sleep")
        rospy.sleep(1)
    def cambiar_position(self):
        position = {'x': -0.006339543795801471,
                        'y': -0.060178817351622366,
                        'z': 0.050735410511438525}

        # Define the orientation
        orientation = {'x': -6.013997907795954e-06,
                    'y': -4.0184256807335854e-05,
                    'z': -0.11672072169886474,
                    'w': 0.9931647755912174}

        # Create a Pose message
        pose_msg = Pose()

        # Set the position
        pose_msg.position.x = position['x']
        pose_msg.position.y = position['y']
        pose_msg.position.z = position['z']

        # Set the orientation
        pose_msg.orientation.x = orientation['x']
        pose_msg.orientation.y = orientation['y']
        pose_msg.orientation.z = orientation['z']
        pose_msg.orientation.w = orientation['w']
        reset_pose_msg = ModelState()
        reset_pose_msg.model_name = 'bebop2'
        reset_pose_msg.pose = pose_msg

        # Assuming reset_pose_publisher is your ROS publisher
        self.reset_pose_publisher.publish(reset_pose_msg)
    def wait_until_twist_achieved(self, cmd_vel_value, epsilon, update_rate):
        """
        We wait for the cmd_vel twist given to be reached by the robot reading
        from the odometry.
        :param cmd_vel_value: Twist we want to wait to reach.
        :param epsilon: Error acceptable in odometry readings.
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        rospy.logdebug("START wait_until_twist_achieved...")

        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0
        epsilon = 0.05

        rospy.logdebug("Desired Twist Cmd>>" + str(cmd_vel_value))
        rospy.logdebug("epsilon>>" + str(epsilon))

        linear_speed = cmd_vel_value.linear.x
        angular_speed = cmd_vel_value.angular.z

        linear_speed_plus = linear_speed + epsilon
        linear_speed_minus = linear_speed - epsilon
        angular_speed_plus = angular_speed + epsilon
        angular_speed_minus = angular_speed - epsilon

        while not rospy.is_shutdown():
            current_odometry = self._check_odom_ready()
            # IN turtlebot3 the odometry angular readings are inverted, so we have to invert the sign.
            odom_linear_vel = current_odometry.twist.twist.linear.x
            odom_angular_vel = -1 * current_odometry.twist.twist.angular.z

            rospy.logdebug("Linear VEL=" + str(odom_linear_vel) + ", ?RANGE=[" + str(linear_speed_minus) + "," + str(
                linear_speed_plus) + "]")
            rospy.logdebug("Angular VEL=" + str(odom_angular_vel) + ", ?RANGE=[" + str(angular_speed_minus) + "," + str(
                angular_speed_plus) + "]")

            linear_vel_are_close = (odom_linear_vel <= linear_speed_plus) and (odom_linear_vel > linear_speed_minus)
            angular_vel_are_close = (odom_angular_vel <= angular_speed_plus) and (
                        odom_angular_vel > angular_speed_minus)

            if linear_vel_are_close and angular_vel_are_close:
                rospy.logdebug("Reached Velocity!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rospy.logdebug("Not there yet, keep waiting...")
            rate.sleep()
        delta_time = end_wait_time - start_wait_time
        rospy.logdebug("[Wait Time=" + str(delta_time) + "]")

        rospy.logdebug("END wait_until_twist_achieved...")

        return delta_time

    def get_odom(self):
        return self.odom

    def get_imu(self):
        return self.imu

    def get_laser_scan(self):
        return self.laser_scan
    
    def get_save(self):
        return self.SAVE


    def get_image_raw(self):
        return self.image_raw
    ##----------------------
    def callback(self, rgb_image1, depth_image1):
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(depth_image1, 'passthrough')
        depth_image = cv2.resize(depth_image, (256, 256))
        depth_array = np.array(depth_image, dtype=np.float32)
		# Normalize the depth image to fall between 0 (black) and 1 (white)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        backtorgb = cv2.cvtColor(depth_array,cv2.COLOR_GRAY2RGB)
        Save_depth = self.model_depth.predict(np.expand_dims((backtorgb*255)/255, 0))
        rgb_image = bridge.imgmsg_to_cv2(rgb_image1, 'passthrough')
        rgb_image = cv2.resize(rgb_image, (256, 256))
        Save_rgb = self.model.predict(np.expand_dims(rgb_image/255, 0))
        if Save_depth > 0.5: #and Save_depth > 0.5:
            self.SAVE=1
        else:
            self.SAVE=0
        self.pub_save.publish(self.SAVE)
        #cv2.imshow('Image Viewer - RGB', backtorgb)
        #cv2.imshow('Image Viewer - DEPTH', rgb_image)
        #cv2.waitKey(1)#print(self.SAVE)
