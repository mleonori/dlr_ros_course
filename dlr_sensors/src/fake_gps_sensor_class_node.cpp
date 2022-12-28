#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/SetBool.h>
#include "dlr_sensors/FakeGPS.h"

namespace dlr{

class FakeGPSNode
{
    public: 
        // Class constructor
        FakeGPSNode() : nh_("~"), active_(false)
        {
            pub_ = nh_.advertise<sensor_msgs::NavSatFix>("data", 10);
            activation_service_ = nh_.advertiseService("activate", &FakeGPSNode::activationServiceCallback, this);
        };

        bool publishData()
        {
            if (active_)
            {
                sensor_msgs::NavSatFix msg;
                msg.header.stamp = ros::Time().now();
                msg.header.frame_id = "gps";
                msg.status.status = sensor_msgs::NavSatStatus().STATUS_FIX;
                //msg.status.service = NavSatStatus.SERVICE_GPS
                msg.latitude = sensorAPIgetLatitude();
                msg.longitude = sensorAPIgetLongitude();
                msg.altitude = sensorAPIgetAltitude();
                
                msg.position_covariance[0] = 0;
                msg.position_covariance[4] = 0;
                msg.position_covariance[8] = 0;
                msg.position_covariance_type = sensor_msgs::NavSatFix().COVARIANCE_TYPE_DIAGONAL_KNOWN;

                ROS_INFO_STREAM("Actual position: [latitude: " << msg.latitude << 
                                        " | longitude: " << msg.longitude << "]");

                pub_.publish(msg);
            }
            
            return true;
        };

        float sensorAPIgetLatitude()
        {
            return 1.5;
        };

        float sensorAPIgetLongitude()
        {
            return 1.5;
        };

        float sensorAPIgetAltitude()
        {
            return 1.5;
        };

    private:

        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::ServiceServer activation_service_;

        bool active_;

        bool activationServiceCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
        {
            if (req.data)
            {
                res.message = "GPS sensor activated.";
                res.success = true;
                ROS_INFO_STREAM(res.message);
                active_ = true;
            }
            else
            {
                res.message = "GPS sensor deactivated.";
                res.success = true;
                ROS_INFO_STREAM(res.message);
                active_ = false;
            }
            return true;
        }

}; // class FakeGPSNode

} // namespace dlr

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps");

    dlr::FakeGPSNode gps_node;

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        gps_node.publishData();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}