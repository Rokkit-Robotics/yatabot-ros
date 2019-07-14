#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import RegionOfInterest, CameraInfo

class RoiTracker(object):
  def __init__(self, node_name):
    rospy.init_node(node_name)

    self.width = None
    self.obj_width = None

    self.roi_topic = rospy.get_param("~roi_topic", "/roi")
    self.caminfo_topic = rospy.get_param("~caminfo_topic", "/yatabot/camera1/camera_info")
    self.twist_topic = rospy.get_param("~twist_topic", "/yatabot_diff_drive_controller/cmd_vel")

    self.p_angular = rospy.get_param("~p_angular", 2.5)
    self.p_linear = rospy.get_param("~p_linear", 0.05)

    self.roi_sub = rospy.Subscriber(self.roi_topic, RegionOfInterest, self.roi_handler)
    self.caminfo_sub = rospy.Subscriber(self.caminfo_topic, CameraInfo, self.camera_info_handler)

    self.pub = rospy.Publisher(self.twist_topic, Twist, queue_size=2)


  def camera_info_handler(self, msg):
    self.width = msg.width


  def roi_handler(self, msg):
    if self.width is None:
      rospy.logwarn("width is still unknown, waiting for camerainfo")
      return

    if msg.width == 0:
      return

    if self.obj_width is None:
      self.obj_width = msg.width

    angular_error = float(self.width / 2 - (msg.x_offset + msg.width / 2)) / self.width

    if self.obj_width > msg.width:
      dist_error = self.obj_width / msg.width - 1.0
    else:
      dist_error = -msg.width / self.obj_width + 1.0

    msg = Twist()
    msg.linear.x = self.p_linear * dist_error
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = self.p_angular * angular_error

    rospy.loginfo("angular error {}, linear {}".format(angular_error, dist_error))

    self.pub.publish(msg)


if __name__ == "__main__":
  RoiTracker('tracker')

  while not rospy.is_shutdown():
    rospy.spin()

