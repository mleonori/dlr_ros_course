#!/usr/bin/env python3

import rospy

if __name__ == '__main__':

    rospy.init_node("time_example", anonymous=False)

    start_time = rospy.Time.now()

    loop_rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        rospy.loginfo("Actual time: " + str(rospy.Time.now()))
        rospy.loginfo("Elapsed time from start_time in seconds: " + 
                      str((rospy.Time.now() - start_time).to_sec()) + "\n")
        
        loop_rate.sleep()
  