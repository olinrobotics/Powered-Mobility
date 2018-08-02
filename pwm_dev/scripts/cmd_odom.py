#!/usr/bin/env python2

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf import transformations as tx
from tf_conversions import posemath as pm

class CmdOdom(object):
    """
    Compute Odometry from cmd_vel.
    WARNING : Only use this node for calibration purposes!!
    """
    def __init__(self):
        # WARNING : no covariance data is populated.
        self._odom_frame = rospy.get_param('~odom_frame', default='odom')
        self._base_frame = rospy.get_param('~base_frame', default='base_footprint')
        self._cmd_timeout = float(rospy.get_param('~cmd_timeout', default=0.5))
        self._rate = rospy.get_param('~rate', default=50)

        self._odom_pub = rospy.Publisher('cmd_odom', Odometry, queue_size=10) 
        self._odom = Odometry(
                header=Header(seq=0, frame_id=self._odom_frame, stamp=rospy.Time.now()),
                child_frame_id=self._base_frame
                )

        self._cmd_vel = Twist()
        self._last_cmd = None
        self._last_update = None

        self._cmd_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_cb)

    def cmd_cb(self, msg):
        self._cmd_vel = msg
        self._last_cmd = rospy.Time.now()

    def update(self, dt=0.0):

        # initial pose
        T0 = pm.toMatrix(pm.fromMsg(self._odom.pose.pose))
        dx = self._cmd_vel.linear
        dq = self._cmd_vel.angular

        dT = tx.compose_matrix(
                angles = (dq.x*dt, dq.y*dt, dq.z*dt),
                translate = (dx.x*dt, dx.y*dt, dx.z*dt)
                )
        T1 = tx.concatenate_matrices(T0, dT)
        self._odom.pose.pose = pm.toMsg(pm.fromMatrix(T1))

    def publish(self, stamp=None):
        if stamp is None:
            stamp = rospy.Time.now()
        self._odom.header.seq += 1
        self._odom.header.stamp = stamp
        self._odom_pub.publish(self._odom)

    def reset(self):
        self._last_cmd = rospy.Time.now()
        self._last_update = rospy.Time.now()
        self._cmd_vel = Twist()
        self._odom.pose.pose = Pose()
        self._odom.pose.pose.orientation.w = 1.0
        self._odom.twist.twist = Twist()

    def step(self):
        now = rospy.Time.now()

        cmd_dt = (now - self._last_cmd).to_sec()
        if cmd_dt > self._cmd_timeout:
            self._cmd_vel = Twist() # reset

        dt = (now - self._last_update).to_sec()
        if dt <= 0:
            return

        self._last_update = now
        self._odom.twist.twist = self._cmd_vel
        self.update(dt)
        self.publish(stamp=now)

    def run(self):
        rate = rospy.Rate(self._rate)
        self.reset()
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

def main():
    rospy.init_node('cmd_odom')
    app = CmdOdom()
    app.run()

if __name__ == "__main__":
    main()
