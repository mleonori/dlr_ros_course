#!/usr/bin/env python3

import rospy 
import tf2_ros
import geometry_msgs.msg
import tf_conversions
import sys

if __name__ == '__main__':
    rospy.init_node('my_tf2_broadcaster')

    if len(sys.argv) != 3:
        rospy.logerr("Provided " + str(len(sys.argv)-1) + " arguments. Need parent and child frame.")
        exit()

    parent_frame = sys.argv[1]
    child_frame = sys.argv[2]

    tf_broadcaster = tf2_ros.TransformBroadcaster()
    transform_stamped = geometry_msgs.msg.TransformStamped()

    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = parent_frame
    transform_stamped.child_frame_id = child_frame
    transform_stamped.transform.translation.x = 0.0
    transform_stamped.transform.translation.y = 0.0
    transform_stamped.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    transform_stamped.transform.rotation.x = q[0]
    transform_stamped.transform.rotation.y = q[1]
    transform_stamped.transform.rotation.z = q[2]
    transform_stamped.transform.rotation.w = q[3]

    loop_rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.transform.translation.x += 0.1
        tf_broadcaster.sendTransform(transform_stamped)
        
        loop_rate.sleep()