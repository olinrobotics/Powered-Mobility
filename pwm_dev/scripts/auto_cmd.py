#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('auto_cmd')
    speed = rospy.get_param('~speed', default=0.0)

    auto_pub = rospy.Publisher('/auto_cmd_vel', Twist, queue_size=10)
    auto_msg = Twist()

    auto_msg.linear.x = speed
    auto_msg.linear.y = 0
    auto_msg.linear.z = 0
    auto_msg.angular.x = 0
    auto_msg.angular.y = 0
    auto_msg.angular.z = 0

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        auto_pub.publish(auto_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
