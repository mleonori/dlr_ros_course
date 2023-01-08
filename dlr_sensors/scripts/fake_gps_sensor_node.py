#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

if __name__ == '__main__':
    
    gps_sensor_pub = rospy.Publisher("gps/data", NavSatFix, queue_size=10)

    rospy.init_node("fake_gps_node", anonymous=True)

    loop_rate = rospy.Rate(1)

    speed = 0.001

    nav_sat_fix_msg = NavSatFix()
    nav_sat_fix_msg.header.frame_id = "world"
    nav_sat_fix_msg.latitude = 52.25444
    nav_sat_fix_msg.longitude = 13.31476

    rospy.loginfo("GPS node started.")

    while not rospy.is_shutdown():
        nav_sat_fix_msg.header.stamp = rospy.Time().now()
        nav_sat_fix_msg.latitude += speed
        nav_sat_fix_msg.longitude -= speed
        
        rospy.loginfo("Actual position: [latitude: " + str(nav_sat_fix_msg.latitude) +
                                     " | longitude: " + str(nav_sat_fix_msg.longitude) + "]")

        gps_sensor_pub.publish(nav_sat_fix_msg);

        loop_rate.sleep()
