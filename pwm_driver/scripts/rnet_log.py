#!/usr/bin/env python2

"""

ROS Binding for the RNet Wheelchair.
Currently supports teleoperation and battery readings; hoping to get odometry via wheel encoders.

To figure out 'non-trivial' rnet messages:
candump can0 -L | grep -Ev '02001100#|02000200#|00E#|03C30F0F#|0C140300#|0C140000#|1C0C0000#|14300000#|1C300004#'
"""

import rospy
import socket, sys, os, array, threading
from fcntl import ioctl
import can
import time
import numpy as np

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
import threading

def dec2hex(dec,hexlen):  #convert dec to hex with leading 0s and no '0x'
    h=hex(int(dec))[2:]
    l=len(h)
    if h[l-1]=="L":
        l-=1  #strip the 'L' that python int sticks on
    if h[l-2]=="x":
        h= '0'+hex(int(dec))[1:]
    return ('0'*hexlen+h)[l:l+hexlen]

def aid_str(msg):
    if msg.id_type:
        return '{0:08x}'.format(msg.arbitration_id)
    else:
        return '{0:03x}'.format(msg.arbitration_id)

def cvt(x):
    if x >= 128:
        x -= 256
    return x
class RNETInterface(object):

    def __init__(self, channel='can0'):
        rospy.init_node('rnet_log')

        # data ...
        self._battery = None

        # joystick
        self._can  = can.interface.Bus(channel=channel, bustype='socketcan_ctypes',
            can_filters=[{"can_id":0x02000200, "can_mask":0x1FFFFFFF, "extended":True}])
        self._can2  = can.interface.Bus(channel=channel, bustype='socketcan_ctypes',
            can_filters=[{"can_id":0x02001100, "can_mask":0x1FFFFFFF, "extended":True}])
        
        self._v1_pub = rospy.Publisher('v1', Int32, queue_size=10)
        self._w1_pub = rospy.Publisher('w1', Int32, queue_size=10)
        self._v2_pub = rospy.Publisher('v2', Int32, queue_size=10)
        self._w2_pub = rospy.Publisher('w2', Int32, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            msg1 = self._can.recv()
            msg2 = self._can2.recv()
            w1,v1 = [cvt(c) for c in msg1.data]
            w2,v2 = [cvt(c) for c in msg2.data]
            self._v1_pub.publish(v1)
            self._w1_pub.publish(w1)
            self._v2_pub.publish(v2)
            self._w2_pub.publish(w2)

if __name__ == "__main__":
    app = RNETInterface()
    app.run()
