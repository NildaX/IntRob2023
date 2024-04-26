#!/usr/bin/env python3
__author__ = "L. Oyuki Rojas-Perez and Dr. Jose Martinez-Carranza"
__license__ = "GPL"
__version__ = "1.0.1"
__maintainer__ = "L. Oyuki Rojas-Perez"
__email__ = "carranza@inaoep.mx"

import rospy
import sys
from getkey import getkey, keys
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import time
class keyboard:
	def __init__(self):
		self.laser_subscriber=rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)
		self.laser_filtered_pub = rospy.Publisher(
            '/bebop/laser/scan_filtered', LaserScan, queue_size=1)
		self.max_laser_value: 6 # Value considered Ok, no wall
    	self.min_laser_value: 0 # Value considered there is an obstacle or crashed
		
	def _laser_scan_callback(self, data):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """
        self._episode_done = False
		new_ranges=50
        discretized_ranges = []
        mod = int(len(data.ranges) / new_ranges)

        rospy.logdebug("data=" + str(data))
        rospy.logdebug("new_ranges=" + str(new_ranges))
        rospy.logdebug("mod=" + str(mod))
        for i, item in enumerate(data.ranges):
            if i % mod == 0:
                if item == float('Inf') or numpy.isinf(item):
                    discretized_ranges.append(round(self.max_laser_value, 2))
                elif numpy.isnan(item):
                    discretized_ranges.append(round(self.max_laser_value, 2))
                else:
                    discretized_ranges.append(round(item, 2))

                if self.min_range > item > 0:
                    rospy.logerr("done Validation >>> item=" + str(item) + "< " + str(self.min_range))
                    self._episode_done = True
                else:
                    rospy.logdebug("NOT done Validation >>> item=" + str(item) + "< " + str(self.min_range))

        self.publish_filtered_laser_scan(laser_original_data=data,new_filtered_laser_range=discretized_ranges)
        return discretized_ranges
	def publish_filtered_laser_scan(self, laser_original_data, new_filtered_laser_range):

        rospy.logdebug("new_filtered_laser_range==>" +
                       str(new_filtered_laser_range))

        laser_filtered_object = LaserScan()

        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = laser_original_data.header.frame_id

        laser_filtered_object.header = h
        laser_filtered_object.angle_min = laser_original_data.angle_min
        laser_filtered_object.angle_max = laser_original_data.angle_max

        new_angle_incr = abs(laser_original_data.angle_max -
                             laser_original_data.angle_min) / len(new_filtered_laser_range)

        laser_filtered_object.angle_increment = new_angle_incr
        laser_filtered_object.time_increment = laser_original_data.time_increment
        laser_filtered_object.scan_time = laser_original_data.scan_time
        laser_filtered_object.range_min = laser_original_data.range_min
        laser_filtered_object.range_max = laser_original_data.range_max

        laser_filtered_object.ranges = []
        laser_filtered_object.intensities = []
        for item in new_filtered_laser_range:
            if item == 0.0:
                laser_distance = 0.1
            else:
                laser_distance = item
            laser_filtered_object.ranges.append(laser_distance)
            laser_filtered_object.intensities.append(item)

        self.laser_filtered_pub.publish(laser_filtered_object)

	def cmd_vel(self):
		rospy.sleep(5)
		key='T'
		while not rospy.is_shutdown():
			
			key_anterior=key
			
			self.pub_cmd_vel.publish(self.vel_msg)
			rospy.sleep(5)
			


def main():
	rospy.init_node('keyboard', anonymous = True)
	# ~ print("init Node keyboard")
	keyboard()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()
