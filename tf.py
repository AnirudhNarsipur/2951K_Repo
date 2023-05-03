#!/usr/bin/env python  
import rospy

import math
import tf2_ros

if __name__ == "__main__":
    TAG_NAME = "tag_0"
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("base", TAG_NAME, rospy.Time())
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error!")
            continue
    print("got tf ",trans)
