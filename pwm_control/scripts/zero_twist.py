#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('estop_pub')

    estop_pub = rospy.Publisher('/estop_cmd_vel', Twist, queue_size=10)
    estop_msg = Twist()

    estop_msg.linear.x = 0
    estop_msg.linear.y = 0
    estop_msg.linear.z = 0
    estop_msg.angular.x = 0
    estop_msg.angular.y = 0
    estop_msg.angular.z = 0

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        estop_pub.publish(estop_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
