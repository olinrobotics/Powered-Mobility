#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Joy, Range
from topic_tools.srv import MuxSelect
from std_msgs.msg import Int32

class SonarStop:
	def __init__(self):
		self._offset = float(rospy.get_param('~dist', default=1.0))
		self._sonar_sub = rospy.Subscriber("/sonar_back", Range, self.callback)
		rospy.wait_for_service('/mux_cmd_vel/select')
		self._srv = rospy.ServiceProxy('/mux_cmd_vel/select', MuxSelect, persistent=True)
		self._last_srv = rospy.Time(0)

	def callback(self,data):
		if data.range < self._offset:
			try:
				now = rospy.Time.now()
				if (now - self._last_srv).to_sec() > 5.0:
					self._srv('estop_cmd_vel')
					self._last_srv = now
			except rospy.ServiceException as exc:
				print("Service did not process request: " + str(exc))

def main():
	rospy.init_node('sonar_stop')
	app = SonarStop()
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == "__main__":#
	main()
