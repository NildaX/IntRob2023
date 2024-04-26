import rospy
import time
from openai_ros_b import robot_gazebo_env
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from openai_ros_b.openai_ros_common import ROSLauncher
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from keras.models import load_model
import message_filters
import cv2
from std_msgs.msg import Int8

class TurtleBot3Env(robot_gazebo_env.RobotGazeboEnv):
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
        rospy.logdebug("Start TurtleBot3Env INIT...")
        # Variables that we give through the constructor.
        # None in this case


        # We launch the ROSlaunch that spawns the robot into the world
        #ROSLauncher(rospackage_name="turtlebot3_gazebo",
        #            launch_file_name="put_robot_in_world.launch",
        #            ros_ws_abspath=ros_ws_abspath)
        ROSLauncher(rospackage_name="turtlebot3_gazebo",
                    launch_file_name="put_robot_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)
        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = ["imu"]

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(TurtleBot3Env, self).__init__(controllers_list=self.controllers_list,
                                            robot_name_space=self.robot_name_space,
                                            reset_controls=False,
                                            start_init_physics_parameters=False)

        self.gazebo.unpauseSim()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        #rospy.Subscriber("/odom", Odometry, self._odom_callback)
        rospy.Subscriber("/odom", Odometry, self._odom_callback)
        rospy.Subscriber("/imu", Imu, self._imu_callback)
        rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)

        rospy.Subscriber("/camera/rgb/image_raw", Image, self._image_rgb_callback)

        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self._check_publishers_connection()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished TurtleBot3Env INIT...")
        
        
        #--------Agregados
        self.bridge = CvBridge()
		# Exportar el modelo a 'SavedModel'
        self.model = load_model('/home/nilda/Turtle/src/openai_ros_b/openai_ros_b/src/openai_ros_b/robot_envs/imageclassifierSaveTwoEnviroments.h5',compile=False)
        self.model_depth = load_model('/home/nilda/Turtle/src/openai_ros_b/openai_ros_b/src/openai_ros_b/robot_envs/imageclassifierDepthTwoEnviroments.h5',compile=False)
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
        self._check_laser_scan_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        self.odom = None
        rospy.logdebug("Waiting for /odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
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

    def _odom_callback(self, data):
        self.odom = data

    def _imu_callback(self, data):
        self.imu = data

    def _laser_scan_callback(self, data):
        self.laser_scan = data

    def _image_rgb_callback(self, data):
        frame_1=self.bridge.imgmsg_to_cv2(data, "bgr8")
        frame_1 = cv2.resize(frame_1, (84, 84))
        self.image_raw_nueva = frame_1
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
    def move_base(self, linear_speed, angular_speed, epsilon=0.05, update_rate=10):
        """
        It will move the base based on the linear and angular speeds given.
        It will wait untill those twists are achived reading from the odometry topic.
        :param linear_speed: Speed in the X axis of the robot base frame
        :param angular_speed: Speed of the angular turning of the robot base frame
        :param epsilon: Acceptable difference between the speed asked and the odometry readings
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        cmd_vel_value = Twist()
        cmd_vel_value.linear.x = linear_speed
        cmd_vel_value.angular.z = angular_speed
        rospy.logdebug("TurtleBot3 Base Twist Cmd>>" + str(cmd_vel_value))
        self._check_publishers_connection()
        self._cmd_vel_pub.publish(cmd_vel_value)
        # self.wait_until_twist_achieved(cmd_vel_value,epsilon,update_rate)
        # Weplace a waitof certain amiunt of time, because this twist achived doesnt work properly
        time.sleep(0.2)

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
