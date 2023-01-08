#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle nh("~");

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tfListener(tf_buffer);

    if (argc != 3)
    {
        ROS_ERROR_STREAM("Provided " << argc-1 << " arguments. Need parent and child frame");
        return -1;
    };

    std::string parent_frame, child_frame;
    parent_frame = argv[1];
    child_frame = argv[2];

    ros::Rate rate(1.0);

    while (ros::ok())
    {
        try
        {
            geometry_msgs::TransformStamped transform_stamped = 
                        tf_buffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
            ROS_INFO("TF received.");
            ROS_INFO_STREAM(transform_stamped);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
        }

        rate.sleep();
    }
    return 0;
};
