#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Joy
from topic_tools.srv import MuxSelect

MANUAL_BUTTON = 2
AUTO_BUTTON = 3
ESTOP_BUTTON = 1

class SelectMode:
    def __init__(self):
        self.select_sub = rospy.Subscriber("/joy", Joy, self.callback)
        self._srv = rospy.ServiceProxy('/mux_cmd_vel/select', MuxSelect)
        self._mode = 'auto'
        self._last_cmd = rospy.Time(0)

    def callback(self,data):
		now = rospy.Time.now()
		if (now - self._last_cmd).to_sec() < 0.1:
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

def main():
    rospy.init_node('select_mode')
    app = SelectMode()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":#
    main()
