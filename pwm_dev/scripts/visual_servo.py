import rospy
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
import numpy as np

class PID(object):
    """ Simple PID controller """
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_i=10.0):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._max_i = max_i
        self._prv_err = 0.0
        self._net_err = 0.0

    def reset(self):
        self._prv_err = 0.0
        self._net_err = 0.0

    def __call__(self, err, dt=0.0):
        if dt <= 0:
            return 0

        # compute control output
        p = self._kp * err
        i = self._ki * self._net_err
        d = (self._prv_err - err) / dt

        # control output
        u = (p + i + d)

        # save persistent data
        self._prv_err = err
        self._net_err += err * dt
        self._net_err = np.clip(self._net_err, -self._max_i, self._max_i)

        return u

class VisualServo(object):
    """ Visual Servoing Node """
    def __init__(self):
        # initialize handles
        rospy.init_node('visual_servo')
        self._sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_cb)
        self.move_pub = rospy.Publisher("/auto_cmd_vel", Twist, queue_size = 10)

        # default setpoint 1m from tag
        self._offset  = rospy.get_param('~offset', default=1.0)
        self._timeout = rospy.get_param('~timeout', default=0.5)
        self._rate    = rospy.get_praam('~rate', default=100)

        # initialize data
        self._tag_x = 0
        self._tag_z = 0

        # instantiate controller
        self._pid_v = PID(kp = 0.1) # TODO : configure kp/ki/kd/max_i
        self._pid_w = PID(kp = 2.0) # TODO : configure kp/ki/kd/max_i

        # keep track of timestamps
        self._last_tag  = rospy.Time(0)
        self._last_ctrl = rospy.Time(0)

    def tag_cb(self, msg):
        tags = msg.detections
        if len(tags) > 0:

            # unwrap data
            tag = tags[0]
            pose_stamped = tag.pose
            pose = pose_stamped.pose
            x = pose.position.x
            z = pose.position.z

            # save data (in polar form)
            self._tag_d = np.sqrt(x**2 + z**2)
            self._tag_a = np.arctan2(-x, z)
            self._last_tag = rospy.Time.now()

    def step(self):
        now = rospy.Time.now()
        move_msg = Twist()

        if (now - self._last_tag).to_sec() > self._timeout:
            # publish == 0
            self.move_pub.publish(move_msg)
        else:
            # compute error
            d_err = (self._tag_d - self._offset)
            a_err = (self._tag_a - 0.0)

            # compute control term
            dt = (now - self._last_ctrl).to_sec()
            move_msg.linear.x  = self._pid_v(d_err, dt)
            move_msg.angular.z = self._pid_w(a_err, dt)
            self.move_pub.publish(move_msg)

        self._last_ctrl = now

    def run(self):
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            self.step() # << where stuff happens
            rate.sleep()

def main():
    app = VisualServo()
    app.run()

if __name__ == "__main__":
    main()
