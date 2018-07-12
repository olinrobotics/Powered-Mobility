#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt16



def talker():
    max_angle = 180
    direction = 1
    angle = 10
    increment = 5
    pub = rospy.Publisher('servo', UInt16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        angle = angle + (increment * direction)
        if(angle > max_angle):
            angle = max_angle
            direction = -direction
        elif(angle < 0):
            angle = 0
            direction = 1
        pub.publish(angle)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
