#!/usr/bin/python2

import tf
import numpy as np
import rospy
from std_msgs.msg import Int32
#from sensor_msgs.msg import Laser

class PubAsTf(object):
	def __init__(self):
		rospy.init_node('pub_as_tf')
		self.l_ang = None
		#self._tfl = tf.TransformListener()
		self._tfb = tf.TransformBroadcaster()
		self._sub = rospy.Subscriber('/lidar_angle', Int32, self.lcb)
	def lcb(self, msg):
		self.l_ang = np.deg2rad(msg.data)
		l = 0.080
		x = l * np.sin(self.l_ang)
		z = l * np.cos(self.l_ang) 
		self._tfb.sendTransform(
			(x,0,z),
			tf.transformations.quaternion_from_euler(0, self.l_ang, 0),
			rospy.Time.now(),
			'laser',
			'base_link'
			)
	def run(self):
		rospy.spin()

if __name__ == "__main__":
	app = PubAsTf()
	app.run()
