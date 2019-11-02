#!/usr/bin/env python

import rospy
import tf
import math

def set_poi_position(event):
  br = tf.TransformBroadcaster()
  ts = event.current_real.to_sec()
  br.sendTransform((0.2 * math.sin(ts), 0.8, 0.1 + 0.1 * math.cos(ts)), tf.transformations.quaternion_from_euler(0, 0, 0), event.current_real, 'point_of_interest', 'world')

if __name__ == "__main__":
  rospy.init_node('poi_test_publisher')

  timer = rospy.Timer(rospy.Duration(0.1), set_poi_position)

  rospy.spin()
