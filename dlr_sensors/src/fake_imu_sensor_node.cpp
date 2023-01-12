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
            return imu_driver_.updateData(elapsed_time);
        }

        bool publishData()
        {
            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time().now();
            msg.header.frame_id = "imu_base_link";
            // Get quaterion
            msg.orientation.w = imu_driver_.getQuaternion().w();
            msg.orientation.x = imu_driver_.getQuaternion().x();
            msg.orientation.y = imu_driver_.getQuaternion().y();
            msg.orientation.z = imu_driver_.getQuaternion().z();
            // Get angular velocities
            msg.angular_velocity.x = imu_driver_.getAngularVelocities()[0];
            msg.angular_velocity.y = imu_driver_.getAngularVelocities()[1];
            msg.angular_velocity.z = imu_driver_.getAngularVelocities()[2];
            // Get angular velocities
            msg.linear_acceleration.x = imu_driver_.getLinearAccelerations()[0];
            msg.linear_acceleration.y = imu_driver_.getLinearAccelerations()[1];
            msg.linear_acceleration.z = imu_driver_.getLinearAccelerations()[2];

            pub_.publish(msg);
            return true;
        };

    private:

        ros::NodeHandle nh_;
        ros::Publisher pub_;
        dlr::FakeIMU imu_driver_;

}; // class FakeIMUNode

} // namespace dlr

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu");

    dlr::FakeIMUNode imu_node;

    float loop_rate_hz = 100.0;
    ros::Rate loop_rate(loop_rate_hz);

    ROS_INFO("IMU node started.");

    while (ros::ok())
    {
        imu_node.updateData(1.0/loop_rate_hz);
        imu_node.publishData();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}