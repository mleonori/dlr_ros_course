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

        bool init()
        {
            return getROSParams();
        }

        bool updateData(const float& elapsed_time)
        {
            return gps_driver_.updateData(elapsed_time);
        }

        bool publishData()
        {
            if (active_)
            {
                if ((pub_.getNumSubscribers() == 0))
                {
                    ROS_WARN_STREAM_THROTTLE(10, "GPS active, but no node is subscribed to \"" << 
                                    nh_.resolveName("data") << "\" ROS topic.");
                    return false;
                }
                sensor_msgs::NavSatFix msg;
                msg.header.stamp = ros::Time().now();
                msg.header.frame_id = frame_id_;
                msg.status.status = sensor_msgs::NavSatStatus().STATUS_FIX;
                //msg.status.service = NavSatStatus.SERVICE_GPS
                msg.latitude = gps_driver_.getLatitude();
                msg.longitude = gps_driver_.getLongitude();
                msg.altitude = gps_driver_.getAltitude();
                
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
        dlr::FakeGPS gps_driver_;

        bool active_;

        float position_covariance_;
        std::string frame_id_;

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

        bool getROSParams()
        {
            // Get a ROS parameter, if does not exists initialization fails
            if (!nh_.getParam("frame_id", frame_id_))
            {
                ROS_ERROR_STREAM("No " << nh_.resolveName("frame_id") << " ROS param found.");
                return false;
            }
            
            // Get a ROS parameter, if does not exists assign a default value
            if (!nh_.param<float>("position_covariance", position_covariance_, 0.005f))
            {
                ROS_WARN_STREAM("No " << nh_.resolveName("position_covariance") << " ROS param found. Assigning default value:" << position_covariance_);
            }
            return true;
        }

}; // class FakeGPSNode

} // namespace dlr

int main(int argc, char** argv)
{
    // Initialize the ROS node 
    ros::init(argc, argv, "gps");

    // Declare GPS object node
    dlr::FakeGPSNode gps_node;

    // GPS node initialization
    if (!gps_node.init())
    {
        ROS_ERROR("Initialization error.");
        return -1;
    }

    // Define the loop rate
    float loop_rate_hz = 1.0;
    ros::Rate loop_rate(loop_rate_hz);

    ROS_INFO("GPS node started.");
    
    while (ros::ok())
    {
        gps_node.updateData(1.0/loop_rate_hz);
        gps_node.publishData();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}