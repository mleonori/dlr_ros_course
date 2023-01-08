#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import sys

if __name__ == '__main__':
    rospy.init_node('my_tf2_listener')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    if len(sys.argv) != 3:
        rospy.logerr("Provided " + str(len(sys.argv)-1) + " arguments. Need parent and child frame.")
        exit()

    parent_frame = sys.argv[1]
    child_frame = sys.argv[2]

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        try:
            transform_stamped = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time())
            rospy.loginfo("TF received.")
            rospy.loginfo(transform_stamped)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("No TF from " + parent_frame + " to " + child_frame + " found.")
        
        rate.sleep()
