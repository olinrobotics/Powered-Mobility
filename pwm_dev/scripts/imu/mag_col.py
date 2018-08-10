import rospy
import numpy as np
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Point, PointStamped

mag_msg = None

class Mag2V3(object):
    """ Magnetic Field Data Collection + RViz Visualization """
    def __init__(self):
        self._sub = rospy.Subscriber('/imu/mag', MagneticField, self.cb)
        self._pub = rospy.Publisher('/imu/magv3', PointStamped, queue_size=10)
        self._pt_msg = PointStamped()
        self._data = []

    def cb(self, msg):
        self._pt_msg.header = msg.header
        self._pt_msg.point.x = msg.magnetic_field.x / 1000.
        self._pt_msg.point.y = msg.magnetic_field.y / 1000.
        self._pt_msg.point.z = msg.magnetic_field.z / 1000.
        x,y,z = self._pt_msg.point.x, self._pt_msg.point.y, self._pt_msg.point.z
        self._data.append( (x,y,z) )
        self._pub.publish(self._pt_msg)

    def save(self):
        np.save('/tmp/mag.npy', self._data)

    def run(self):
        rate = rospy.Rate(50)
        rospy.on_shutdown(self.save)
        while not rospy.is_shutdown():
            rate.sleep()

def main():
    rospy.init_node('mag2v3')
    app = Mag2V3()
    app.run()

if __name__ == "__main__":
    main()
