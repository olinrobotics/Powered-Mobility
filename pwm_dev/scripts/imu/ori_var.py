import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
import tf_conversions
import tf
import rospy

def cvar(x):
    r = np.sqrt(np.square(np.sum(np.cos(x))) +
            np.square(np.sum(np.sin(x))))
    return 1.0 - (r/np.size(x))

class OriVar(object):
    def __init__(self):
        self._sub = rospy.Subscriber('imu', Imu, self.cb)
        self._data = []
    def cb(self, msg):
        pose = Pose(orientation=msg.orientation)
        _, q = tf_conversions.toTf(tf_conversions.fromMsg(pose))
        rpy = tf.transformations.euler_from_quaternion(q)
        print rpy
        self._data.append(rpy)
    def show(self):
        rx, ry, rz = np.transpose(self._data)
        print cvar(rx), cvar(ry), cvar(rz)
    def run(self):
        rospy.on_shutdown(self.show)
        rospy.spin()

def main():
    rospy.init_node('ori_var')
    app = OriVar()
    app.run()

if __name__ == "__main__":
    main()
