#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/SetBool.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void GPSDataCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    ROS_INFO_STREAM("GPS Data received: latitude " << msg->latitude << " - longitude " << msg->longitude);
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "gps_logger");

    // Declare the ROS node handle
    ros::NodeHandle nh;

    // Get the GPS activation service server name
    std::string gps_activation_srv_name;
    nh.param<std::string>("gps_activation_srv_name", gps_activation_srv_name, "gps/activate");

    // Define the GPS activation client
    ros::ServiceClient gps_activation_client = nh.serviceClient<std_srvs::SetBool>(gps_activation_srv_name);

    // Wait for the GPS activation service
    ROS_WARN_STREAM("Waiting for \"" << nh.resolveName(gps_activation_srv_name) << "\" ROS service...");
    gps_activation_client.waitForExistence();
    ROS_INFO_STREAM("\"" << nh.resolveName(gps_activation_srv_name) << "\" ROS service available.");    

    // Activate the GPS
    std_srvs::SetBool gps_activation_srv;
    gps_activation_srv.request.data = true;
    if (!gps_activation_client.call(gps_activation_srv))
    {
        ROS_ERROR("Error calling the GPS service activation");
        return -1;
    }
    else
    {
        if (gps_activation_srv.response.success)
            ROS_INFO_STREAM(gps_activation_srv.response.message);
        else
            ROS_ERROR_STREAM(gps_activation_srv.response.message);
    }


    ros::Subscriber gps_sensor_sub = nh.subscribe("gps/data", 1000, GPSDataCallback);

    ros::spin();

    return 0;
}