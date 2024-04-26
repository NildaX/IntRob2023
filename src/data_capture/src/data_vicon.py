#!/usr/bin/env python3
__author__ = "L. Oyuki Rojas-Perez and Dr. Jose Martinez-Carranza"
__license__ = "GPL"
__version__ = "1.0.1"
__maintainer__ = "L. Oyuki Rojas-Perez"
__email__ = "carranza@inaoep.mx"

import rospy
from geometry_msgs.msg import TransformStamped

class vicon_data:
	def __init__(self):
		
		self.vicon = rospy.Subscriber('/vicon/drone_bebop/drone_bebop', TransformStamped, self.vicon_pose)
		
	def vicon_pose(self,data):
		
		self.pose = data.transform.translation
		self.rot = data.transform.rotation
		
		print('')
		print('============== Position ==================')
		print('x:', self.pose.x)
		print('y:', self.pose.y)
		print('z:', self.pose.z)
		print('============== Orientation Quaternion ==================')
		print('qx:', self.rot.x)
		print('qy:', self.rot.y)
		print('qz:', self.rot.z)
		print('qw:', self.rot.w)

def main():
	rospy.init_node('Vicon_Data', anonymous = True)
	print("init Node Vicon Data")
	vicon_data()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()
		sys.exit()

if __name__ == '__main__':
	main()
