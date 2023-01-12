#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import SetBool, SetBoolResponse

class FakeGPSNode:
    def __init__(self, activation_service_name="activate"):
        self.__active = False
        self.__activation_service_name = "~"+activation_service_name
        self.__pub = rospy.Publisher("~data", NavSatFix, queue_size=10)
        self.__activation_service = rospy.Service(self.__activation_service_name, SetBool, self.activation_service_callback)
        self.__latitude = 52.25444
        self.__longitude = 13.31476
        self.__altitude = 12.345
        self.__constant_speed = 0.001
        self.__frame_id = "gps_base_link"
        self.__position_covariance = 0.0

    def init(self):
        return self.get_ROS_params()

    def update_data(self, elapsed_time):
        self.__latitude += (elapsed_time * self.__constant_speed)
        self.__longitude -= (elapsed_time * self.__constant_speed)
        self.__altitude = self.__altitude
        return True

    def publish_data(self):
        if self.__active:
            if self.__pub.get_num_connections() == 0:
                rospy.logwarn_throttle(10, "GPS active, but no node is subscribed to \"" +
                                    rospy.resolve_name("data") + "\" ROS topic.")
                return False
            
            msg = NavSatFix()
            msg.header.stamp = rospy.Time().now()
            msg.header.frame_id = self.__frame_id
            msg.status.status = NavSatStatus().STATUS_FIX
            #msg.status.service = NavSatStatus().SERVICE_GPS
            msg.latitude = self.__latitude
            msg.longitude = self.__longitude
            msg.altitude = self.__altitude
                
            msg.position_covariance[0] = self.__position_covariance
            msg.position_covariance[4] = self.__position_covariance
            msg.position_covariance[8] = self.__position_covariance
            msg.position_covariance_type = NavSatFix().COVARIANCE_TYPE_DIAGONAL_KNOWN

            rospy.loginfo("Actual position: [latitude: " + str(msg.latitude) + 
                                    " | longitude: " + str(msg.longitude) + "]")

            self.__pub.publish(msg)
        else:
            rospy.logwarn_throttle(10, "Fake GPS sensor node waiting for activation. Call \"" +
                                        rospy.resolve_name(self.__activation_service_name) + "\" service using the following command: " +
                                        "rosservice call " + rospy.resolve_name(self.__activation_service_name) + " \"data: true\"")
            return False
        
        return True

    def activation_service_callback(self, req):
        res = SetBoolResponse()
        if req.data:
            res.message = "GPS sensor activated."
            res.success = True
            rospy.loginfo(res.message)
            self.__active = True
        else:
            res.message = "GPS sensor deactivated."
            res.success = True
            rospy.loginfo(res.message)
            self.__active = False
        return res

    def get_ROS_params(self):
        # Get a ROS parameter, if does not exists initialization fails
        try:
            self.__frame_id = rospy.get_param("~frame_id")
        except:
            rospy.logerr("No " + rospy.resolve_name("frame_id") + " ROS param found.")
            return False
        
        # Get a ROS parameter, if does not exists assign a default value
        try:
            self.__position_covariance = rospy.get_param("position_covariance", 0.005)
        except:
            rospy.logwarn("No " + rospy.resolve_name("position_covariance") + " ROS param found. Assigning default value:" + self.__position_covariance);

        return True

if __name__ == '__main__':

    rospy.init_node("gps", anonymous=False)

    gps_node = FakeGPSNode()

    if not gps_node.init():
        rospy.logerr("Initialization error.")
        exit()

    loop_rate_hz = 1.0
    loop_rate = rospy.Rate(loop_rate_hz)

    rospy.loginfo("GPS node started.")

    while not rospy.is_shutdown():

        gps_node.update_data(1.0/loop_rate_hz)
        gps_node.publish_data()

        loop_rate.sleep()