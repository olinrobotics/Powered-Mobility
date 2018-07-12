from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
import tf

class image_converter:


  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)
    self.depth_sub = rospy.Subscriber("/camera/depth_registered/image", Image, self.depth_callback)
    self.depth_reg_sub = rospy.Subscriber("camera/depth_registered/camera_info", CameraInfo, self.depth_reg_callback)
    self.drawing = False
    self.depth_image = None
    self.ix = -1
    self.iy = -1
    self.cameramodel = PinholeCameraModel()
    self.pose_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    self._frame_id = None
    self.transform_listener = tf.TransformListener()

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("image", cv_image)
    cv2.setMouseCallback('image',self.mouseclick)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def depth_callback(self,data):
      try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
      except CvBridgeError as e:
        print(e)
      self.depth_image = cv_image

  def depth_reg_callback(self,data):
      self._frame_id = data.header.frame_id
      self.cameramodel.fromCameraInfo(data)

  def mouseclick(self, event, x, y, flags, param):
      if event == cv2.EVENT_LBUTTONDOWN:
          self.drawing = True
          self.ix = x
          self.iy = y

          if self.depth_image is None:
              return

          print(x,y)
          print(self.depth_image[y,x])
          print(self.cameramodel.projectPixelTo3dRay((x,y)))
          point = np.asarray(self.depth_image[y,x])*self.cameramodel.projectPixelTo3dRay((x,y))

          msg = PoseStamped()

          msg.header.frame_id = self._frame_id
          msg.header.stamp = rospy.Time.now()

          msg.pose.position.x = point[0]
          msg.pose.position.y = point[1]
          msg.pose.position.z = point[2]

          try:
              msg = (self.transform_listener.transformPose("/camera_link", msg))
          except tf.Exception as e:
              return

          angle = np.arctan2(msg.pose.position.y, msg.pose.position.x)

          msg.pose.orientation.x = 0.0
          msg.pose.orientation.y = 0.0
          msg.pose.orientation.z = np.sin(angle/2)
          msg.pose.orientation.w = np.cos(angle/2)

          self.pose_pub.publish(msg)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
