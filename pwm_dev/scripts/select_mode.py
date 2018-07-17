#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Joy
from topic_tools.srv import MuxSelect
from std_msgs.msg import Int32

MANUAL_BUTTON = 2
AUTO_BUTTON = 3
ESTOP_BUTTON = 1
LB_BUTTON = 4
RB_BUTTON = 5

class SelectMode:
	def __init__(self):
		self.select_sub = rospy.Subscriber("/joy", Joy, self.callback)
		self._srv = rospy.ServiceProxy('/mux_cmd_vel/select', MuxSelect)
		self._mode = 'auto'
		self._last_cmd = rospy.Time(0)
		self.manual_speed = 30
		self.manual_pub = rospy.Publisher("manual_speed", Int32, queue_size = 10)

	def callback(self,data):
		now = rospy.Time.now()
		if (now - self._last_cmd).to_sec() < 0.2:
			return

		if data.buttons[MANUAL_BUTTON]:
			self._mode = 'manual'
			self._srv('manual_cmd_vel')
			self._last_cmd = now

		if data.buttons[AUTO_BUTTON]:
			self._mode = 'auto'
			self._srv("auto_cmd_vel")
			self._last_cmd = now

		if data.buttons[ESTOP_BUTTON]:
			self._mode = 'estop'
			self._srv('estop_cmd_vel')
			self._last_cmd = now

		if self._mode == 'manual':
			change = 5
			buttonPressed = False
			if data.buttons[LB_BUTTON]:
					self.manual_speed -= change
					buttonPressed = True
			if data.buttons[RB_BUTTON]:
					self.manual_speed += change
					buttonPressed = True
			if buttonPressed:
				self._last_cmd = now
				if self.manual_speed <= 0:
						self.manual_speed = 0
				if self.manual_speed > 100:
						self.manual_speed = 100
				#print(self.manual_speed)
				self.manual_pub.publish(Int32(self.manual_speed))

def main():
	rospy.init_node('select_mode')
	app = SelectMode()
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == "__main__":#
	main()
