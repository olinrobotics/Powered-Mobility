import message_filters

"""
cmd_vel to wheelchair input calibration.
TODO : finish implementation
"""

"""
class CmdCalib(object):

    def __init__(self):
        self._slop = 0.1
        co_sub = message_filters.Subscriber('/cmd_odom', Odometry)
        vo_sub = message_filters.Subscriber('/vis_odom', Odometry)
        self._sync = message_filters.ApproximateTimeSynchronizer(
                [cmd_odom_sub, vis_odom_sub], 10, slop, allow_headerless=False)
        self._sync.registerCallback(self.odom_cb)

        self._v_scale = 1.0
        self._w_scale = 1.0
        self._v_sample = [0.2, 0.4, 0.6, 0.8, 1.0]
        self._w_sample = [0.2, 0.4, 0.6, 0.8, 1.0]

    def odom_cb(self, cmd_odom, vis_odom):
"""
