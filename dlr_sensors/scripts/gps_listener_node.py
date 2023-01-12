#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool, SetBoolRequest

# This tutorial demonstrates simple receipt of messages over the ROS system.

def gps_data_callback(msg):
    rospy.loginfo("GPS Data received: latitude " + str(msg.latitude) + " - longitude " + str(msg.longitude))

if __name__ == '__main__':

    rospy.init_node("gps_listener_node", anonymous=False)

    # Get the GPS activation service server name
    gps_activation_srv_name = rospy.get_param("gps_activation_srv_name", "gps/activate")

    try:
        # Define the GPS activation client
        gps_activation_client = rospy.ServiceProxy(gps_activation_srv_name, SetBool)

        # Wait for the GPS activation service
        rospy.logwarn("Waiting for \"" + rospy.resolve_name(gps_activation_srv_name) + "\" ROS service...")
        gps_activation_client.wait_for_service()
        rospy.loginfo("\"" + rospy.resolve_name(gps_activation_srv_name) + "\" ROS service available.")  

        # Activate the GPS
        gps_activation_req = SetBoolRequest()
        gps_activation_req.data = True
        gps_activation_res = gps_activation_client(gps_activation_req)

        if gps_activation_res.success:
            rospy.loginfo(gps_activation_res.message)
        else:
            rospy.logerr(gps_activation_res.message)
            exit()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        rospy.logerr("Error calling the GPS service activation");
        exit()

    gps_sensor_sub = rospy.Subscriber("gps/data", NavSatFix, gps_data_callback, queue_size=10)

    rospy.spin()

    exit()