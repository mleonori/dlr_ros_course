#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/SetBool.h>
#include "dlr_sensors/FakeGPS.h"

namespace dlr{

class FakeGPSNode
{
    public: 
        // Class constructor
        FakeGPSNode() : nh_("~"), active_(false), activation_service_name_("activate")
        {
            pub_ = nh_.advertise<sensor_msgs::NavSatFix>("data", 10);
            activation_service_ = nh_.advertiseService(activation_service_name_, &FakeGPSNode::activationServiceCallback, this);
        };

        bool updateData(const float& elapsed_time)
        {
            return gps_.updateData(elapsed_time);
        }

        bool publishData()
        {
            if (active_)
            {
                sensor_msgs::NavSatFix msg;
                msg.header.stamp = ros::Time().now();
                msg.header.frame_id = "gps";
                msg.status.status = sensor_msgs::NavSatStatus().STATUS_FIX;
                //msg.status.service = NavSatStatus.SERVICE_GPS
                msg.latitude = gps_.getLatitude();
                msg.longitude = gps_.getLongitude();
                msg.altitude = gps_.getAltitude();
                
                msg.position_covariance[0] = 0;
                msg.position_covariance[4] = 0;
                msg.position_covariance[8] = 0;
                msg.position_covariance_type = sensor_msgs::NavSatFix().COVARIANCE_TYPE_DIAGONAL_KNOWN;

                ROS_INFO_STREAM("Actual position: [latitude: " << msg.latitude << 
                                        " | longitude: " << msg.longitude << "]");

                pub_.publish(msg);
            }
            else
            {
                ROS_WARN_STREAM_THROTTLE(10, "Fake GPS sensor node waiting for activation. Call \"" << 
                                        nh_.resolveName(activation_service_name_) << "\" service using the following command: " <<
                                        "rosservice call " << nh_.resolveName(activation_service_name_) << " \"data: true\"");
            }
            
            return true;
        };

    private:

        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::ServiceServer activation_service_;
        std::string activation_service_name_;
        dlr::FakeGPS gps_;

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

    float loop_rate_hz = 1.0;
    ros::Rate loop_rate(loop_rate_hz);

    while (ros::ok())
    {
        gps_node.updateData(1.0/loop_rate_hz);
        gps_node.publishData();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}