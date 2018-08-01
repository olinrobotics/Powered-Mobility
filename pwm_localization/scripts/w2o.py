#!/usr/bin/env python2

import rospy
import tf
import tf_conversions.posemath as pm
from tf import transformations as tx
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from geometry_msgs.msg import Twist, Pose, Vector3
from geometry_msgs.msg import TwistWithCovariance, PoseWithCovariance
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped

class WorldToOdom(object):
    def __init__(self):
        self._tfl = tf.TransformListener()
        self._tfb = tf.TransformBroadcaster()
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        #self._odom_pub = rospy.Publisher('/odom_map', Odometry, queue_size=10)
        self._timeout = rospy.Duration(0.5)

    def odom_cb(self, msg):
        stamp = msg.header.stamp

        try:
            self._tfl.waitForTransform('base_footprint', 'odom', stamp, timeout=self._timeout)
            o2b_p, o2b_q = self._tfl.lookupTransform('odom', 'base_footprint', stamp)#'base_footprint', 'odom', stamp)
        except tf.Exception as e:
            rospy.loginfo('w2o tf error  : {}'.format(e))
            return

        o2b_T = tx.concatenate_matrices(
                tx.translation_matrix(o2b_p), tx.quaternion_matrix(o2b_q)
                )
        o2b_T_i = tx.inverse_matrix(o2b_T)

        w2b = msg.pose.pose
        w2b_T = pm.toMatrix(pm.fromMsg(w2b))

        w2o_T = tx.concatenate_matrices(w2b_T, o2b_T_i)
        
        txn, rxn = pm.toTf(pm.fromMatrix(w2o_T))

        self._tfb.sendTransform(txn, rxn, stamp, 'odom', 'world')

        # try:
        #     self._tfl.waitForTransform('map', 'world', stamp, timeout=self._timeout)
        #     m2w_p, m2w_q = self._tfl.lookupTransform('map', 'world', stamp)
        #     m2w_T = tx.concatenate_matrices(
        #         tx.translation_matrix(m2w_p), tx.quaternion_matrix(m2w_q)
        #         )

        #     m2b_T = tx.concatenate_matrices(m2w_T, w2b_T)
        #     p = pm.toMsg(pm.fromMatrix(m2b_T))
        #     #p = self._tfl.transformPose('map', PoseStamped(header=msg.header, pose=msg.pose.pose)).pose
        #     #v = self._tfl.transformVector3('map', Vector3Stamped(header=msg.header, vector=msg.twist.twist.linear)).vector
        #     #w = self._tfl.transformVector3('map', Vector3Stamped(header=msg.header, vector=msg.twist.twist.angular)).vector
        # except tf.Exception as e:
        #     rospy.loginfo('w2o odom tf error : {}'.format(e))
        #     return
        # odom_out = Odometry(
        #         header = Header(seq=msg.header.seq, stamp=msg.header.stamp, frame_id='map'),
        #         child_frame_id='base_footprint',
        #         pose = PoseWithCovariance(pose=p),
        #         )
        # self._odom_pub.publish(odom_out)

    def run(self):
        rospy.spin()

def main():
    rospy.init_node('world_to_odom')
    app = WorldToOdom()
    app.run()

if __name__ == "__main__":
    main()
