#include <ros/ros.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "time_example");

    ros::NodeHandle nh;

    ros::Time start_time = ros::Time::now();

    ros::Rate loop_rate(0.5);

    while(ros::ok())
    {
        ROS_INFO_STREAM("Actual time: " << ros::Time::now());
        ROS_INFO_STREAM("Elapsed time from start_time in seconds: " << (ros::Time::now() - start_time).toSec() << "\n");
        
        loop_rate.sleep();
    }

    return 0;
}

