import rospy
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
import numpy as np

class VisualServo(object):
	def __init__(self):
		rospy.init_node('visual_servo')
		self._sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_cb)
		self.move_pub = rospy.Publisher("/auto_cmd_vel", Twist, queue_size = 10)
		self.tag_x = 0
		self.tag_z = 0

	def tag_cb(self, msg):
		tags = msg.detections
		if len(tags) > 0:
			tag = tags[0]
			pose_stamped = tag.pose
			pose = tag.pose.pose
			self.tag_x = pose.position.x
			self.tag_z = pose.position.z

	def step(self):
		z_dist = 0.5

		tag_angle_offset = np.arctan2(self.tag_x, self.tag_z)
		tag_angle_threshold = np.deg2rad(5)

		move_msg = Twist()

		if np.abs(tag_angle_offset) >= tag_angle_threshold or self.tag_z >= z_dist:
			move_msg.angular.z = - 0.1 * tag_angle_offset
			move_msg.linear.x = 0.1 * self.tag_x

		self.move_pub.publish(move_msg)

	def run(self):
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			self.step() # << where stuff happens
			rate.sleep()

def main():
	app = VisualServo()
	app.run()

if __name__ == "__main__":
	main()
