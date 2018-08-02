#!/usr/bin/env python2

import rospy
import message_filters
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class CmdCalib(object):
    """
    Learn the nonlinear mapping from current cmd_vel estimate to actual velocity.
    Note that caster wheel hysteresis is not modeled, which may cause erroneous calibration.
    For best results, it is suggested to hold the each command at a steady-state,
    to prevent transient behavior from dominating the calibration results.
    """

    def __init__(self):
        self._slop = rospy.get_param('~slop', default=0.01)
        cv_sub = message_filters.Subscriber('/cmd_vel', Twist)
        vo_sub = message_filters.Subscriber('/odom', Odometry) # must be filtered odometry!
        self._sub = message_filters.ApproximateTimeSynchronizer(
                [cv_sub, vo_sub], 10, self._slop, allow_headerless=True)
        self._sub.registerCallback(self.data_cb)
        self._data = [] # == (cv,cw,ev,ew,vv,vw)

    def data_cb(self, cmd_vel, vis_odom):
        cv,cw  = cmd_vel.linear.x, cmd_vel.angular.z
        ev,ew  = vis_odom.twist.twist.linear.x, vis_odom.twist.twist.angular.z
        # variance (weighing factor for later)
        vv     = vis_odom.twist.covariance[0*6+0]
        vw     = vis_odom.twist.covariance[5*6+5]

        entry = (cv,cw,ev,ew,vv,vw)
        rospy.loginfo_throttle(1.0, '{}'.format(entry))
        self._data.append(entry)

    def save(self):
        data = np.asarray(self._data, dtype=np.float32)
        np.save('/tmp/data.npy', data)

    def run(self):
        rospy.on_shutdown(self.save)
        rospy.spin()

def main():
    rospy.init_node('cmd_calib')
    app = CmdCalib()
    app.run()

if __name__ == '__main__':
    main()
