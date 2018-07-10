import rospy
from sensor_msgs.msg import Joy
from topic_tools.srv import MuxSelect

class SelectMode:
    def __init__(self):
        self.select_sub = rospy.Subscriber("/joy", Joy, self.callback)
        self._srv = rospy.ServiceProxy('/mux_cmd_vel/select', MuxSelect)
        self._mode = 'auto'
        self._last_cmd = rospy.Time(0)

    def callback(self,data):
        if data.buttons[1] == True:
            now = rospy.Time.now()
            if (now - self._last_cmd).to_sec() < 0.1:
                return
            self._last_cmd = now
            if self._mode == 'auto':
                self._mode = 'estop'
                self._srv("estop_cmd_vel")
            else:
                self._mode = 'auto'
                self._srv("auto_cmd_vel")


def main():
    rospy.init_node('select_mode')
    app = SelectMode()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":#
    main()
