#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "dlr_sensors/FakeIMU.h"

namespace dlr{

class FakeIMUNode
{
    public: 
        // Class constructor
        FakeIMUNode() : nh_("~")
        {
            pub_ = nh_.advertise<sensor_msgs::Imu>("data", 10);
        };

        bool updateData(const float& elapsed_time)
        {
            return imu_.updateData(elapsed_time);
        }

        bool publishData()
        {
            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time().now();
            msg.header.frame_id = "imu";
            //msg.status.service = NavSatStatus.SERVICE_GPS
            // msg.latitude = gps_.getLatitude();
            // msg.longitude = gps_.getLongitude();
            // msg.altitude = gps_.getAltitude();
            
            // msg.position_covariance[0] = 0;
            // msg.position_covariance[4] = 0;
            // msg.position_covariance[8] = 0;
            // msg.position_covariance_type = sensor_msgs::NavSatFix().COVARIANCE_TYPE_DIAGONAL_KNOWN;

            pub_.publish(msg);
            return true;
        };

    private:

        ros::NodeHandle nh_;
        ros::Publisher pub_;
        dlr::FakeIMU imu_;

}; // class FakeIMUNode

} // namespace dlr

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu");

    dlr::FakeIMUNode imu_node;

    float loop_rate_hz = 100.0;
    ros::Rate loop_rate(loop_rate_hz);

    while (ros::ok())
    {
        imu_node.updateData(1.0/loop_rate_hz);
        imu_node.publishData();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}