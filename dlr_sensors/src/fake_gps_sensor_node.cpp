#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char** argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "fake_gps_sensor");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle nh;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher gps_sensor_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/data", 10);

    ros::Rate loop_rate(1);

    double speed = 0.001;

    sensor_msgs::NavSatFix nav_sat_fix_msg;
    nav_sat_fix_msg.header.frame_id = "world";
    nav_sat_fix_msg.latitude = 52.25444;
    nav_sat_fix_msg.longitude = 13.31476;

    ROS_INFO("GPS node started.");

    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        nav_sat_fix_msg.header.stamp = ros::Time().now();
        nav_sat_fix_msg.latitude += speed;
        nav_sat_fix_msg.longitude -= speed;
        
        ROS_INFO_STREAM("Actual position: [latitude: " << nav_sat_fix_msg.latitude << 
                                       " | longitude: " << nav_sat_fix_msg.longitude << "]");

        /**
        * The publish() function is how you send messages. The parameter
        * is the message object. The type of this object must agree with the type
        * given as a template parameter to the advertise<>() call, as was done
        * in the constructor above.
        */
        gps_sensor_pub.publish(nav_sat_fix_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}