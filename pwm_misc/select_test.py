import rospy
from sensor_msgs.msg import Joy
from topic_tools.srv import MuxSelect

class be_independent:
    def __init__(self):
        self.select_sub = rospy.Subscriber("/joy", Joy, self.callback)
        self._srv = rospy.ServiceProxy('/mux_cmd_vel/select', MuxSelect)
        self._mode = 'auto'

    def callback(self,data):
        if data.buttons[-1] == True:
            if self._mode == 'auto':
                self._mode = 'manual'
                self._srv("manual_cmd_vel")
            else:
                self._mode = 'auto'
                self._srv("auto_cmd_vel")


def main():
    rospy.init_node('yyyyyy')
    app = be_independent()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":#
    main()
