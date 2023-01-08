#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf2_broadcaster");

    ros::NodeHandle nh("~");

    if (argc != 3)
    {
        ROS_ERROR_STREAM("Provided " << argc-1 << 
             " arguments. Need parent and child frame");
        return -1;
    };

    std::string parent_frame, child_frame;
    parent_frame = argv[1];
    child_frame = argv[2];
        
    tf2_ros::TransformBroadcaster tf_broadcaster;

    geometry_msgs::TransformStamped transform_stamped;
    
    transform_stamped.header.frame_id = parent_frame;
    transform_stamped.child_frame_id = child_frame;
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    ros::Rate loop_rate(1.0);

    while (ros::ok())
    {
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.transform.translation.x += 0.1;
        tf_broadcaster.sendTransform(transform_stamped);
        
        ros::spinOnce();
        loop_rate.sleep();
    }   

    return 0;
};


