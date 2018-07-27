#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Int32
import numpy as np

def main():
    rospy.init_node('rotate_lidar')
    rate = rospy.get_param('~rate', default=50.0)
    rate_rotate = rospy.get_param('~rate_rotate', default=5.0)

    angle_min = rospy.get_param('~angle_min', default=30.0)
    angle_max = rospy.get_param('~angle_max', default=50.0)

    angle_min = np.deg2rad(angle_min)
    angle_max = np.deg2rad(angle_max)

    #pub = rospy.Publisher('/pwm/lidar_controller/command', Float64, queue_size=10)
    pub = rospy.Publisher('/lidar_angle', Int32, queue_size=10)

    rate = rospy.Rate(rate)
    #msg = Float64()
    msg = Int32()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        theta = (angle_max - angle_min) * np.sin(2*np.pi*rate_rotate*t) + (angle_min)
        msg.data = theta
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
