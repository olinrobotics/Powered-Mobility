import rospy
from sensor_msgs.msg import Joy, Range
from topic_tools.srv import MuxSelect
from std_msgs.msg import Int32

class SonarStop:
	def __init__(self):
		self._sonar_sub = rospy.Subscriber("/sonar_front", Range, self.callback)

		rospy.wait_for_service('/mux_cmd_vel/select')
		self._srv = rospy.ServiceProxy('/mux_cmd_vel/select', MuxSelect, persistent=True)

	def callback(self,data):
		if data.range < .5:
			try:
				self._srv('estop_cmd_vel')
			except rospy.ServiceException as exc:
				print("Service did not process request: " + str(exc))

def main():
	rospy.init_node('select_mode')
	app = SonarStop()
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == "__main__":#
	main()
