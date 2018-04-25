import rospy
from std_msgs.msg import Int32

def main():
	rospy.init_node('lidar_swivel')
	pub = rospy.Publisher('lidar_angle', Int32, queue_size=1)
	rate = rospy.Rate(60)

	while not rospy.is_shutdown():
		for i in range(10, 80,  1):
			pub.publish(i)
			rate.sleep()
		for i in range(80, 10, -1):
			pub.publish(i)
			rate.sleep()

if __name__ == "__main__":
	main()
