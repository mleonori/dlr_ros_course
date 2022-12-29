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

        bool init()
        {
            return getROSParams();
        }

        bool updateData()
        {
            return gps_driver_.updateData();
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
            return gps_driver_.getLatitude();
        };

        float sensorAPIgetLongitude()
        {
            return gps_driver_.getLongitude();
        };

        float sensorAPIgetAltitude()
        {
            return gps_driver_.getAltitude();
        };

        bool isActive()
        {
            return active_;
        }

    private:

        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::ServiceServer activation_service_;
        std::string frame_id_;
        float position_covariance_;

        dlr::FakeGPS gps_driver_;

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

        bool getROSParams()
        {
            // Get a ROS parameter, if does not exists initialization fails
            if (!nh_.getParam("frame_id", frame_id_))
            {
                ROS_ERROR_STREAM("No " << nh_.resolveName("frame_id") << " ROS param found.");
                return false;
            }
            
            // Get a ROS parameter, if does not exists assign a default value
            position_covariance_
            if (!nh_.param<float>("position_covariance", position_covariance_, 0.005))
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
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        if (gps_node.isActive())
        {
            gps_node.updateData();
            gps_node.publishData();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}