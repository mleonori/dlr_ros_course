
To validate your bag
```
cartographer_rosbag_validate -bag_filename your_bag.bag
```
Among the values you need to adapt, you probably have to provide the TF frame IDs of your environment and robot in map_frame, tracking_frame, published_frame and odom_frame.

The other values you need to define are related to the number and type of sensors you would like to use.

    num_laser_scans: Number of sensor_msgs/LaserScan topics you’ll use.
    num_multi_echo_laser_scans: Number of sensor_msgs/MultiEchoLaserScan topics you’ll use.
    num_point_clouds: Number of sensor_msgs/PointCloud2 topics you’ll use.

You can also enable the usage of landmarks and GPS as additional sources of localization using use_landmarks and use_nav_sat. The rest of the variables in the options block should typically be left untouched.

However, there is one global variable that you absolutely need to adapt to the needs of your bag: TRAJECTORY_BUILDER_3D.num_accumulated_range_data or TRAJECTORY_BUILDER_2D.num_accumulated_range_data. This variable defines the number of messages required to construct a full scan (typically, a full revolution). If you follow cartographer_rosbag_validate’s advices and use 100 ROS messages per scan, you can set this variable to 100. If you have two range finding sensors (for instance, two LIDARs) providing their full scans all at once, you should set this variable to 2.
Create .launch files for your SLAM scenarios

You may have noticed that each demo introduced in the previous section was run with a different roslaunch command. The recommended usage of Cartographer is indeed to provide a custom .launch file per robot and type of SLAM. The example .launch files are defined in src/cartographer_ros/cartographer_ros/launch and installed in install_isolated/share/cartographer_ros/launch/.

Start by copying one of the provided example:

cp install_isolated/share/cartographer_ros/launch/backpack_3d.launch install_isolated/share/cartographer_ros/launch/my_robot.launch
cp install_isolated/share/cartographer_ros/launch/demo_backpack_3d.launch install_isolated/share/cartographer_ros/launch/demo_my_robot.launch
cp install_isolated/share/cartographer_ros/launch/offline_backpack_3d.launch install_isolated/share/cartographer_ros/launch/offline_my_robot.launch
cp install_isolated/share/cartographer_ros/launch/demo_backpack_3d_localization.launch install_isolated/share/cartographer_ros/launch/demo_my_robot_localization.launch
cp install_isolated/share/cartographer_ros/launch/assets_writer_backpack_3d.launch install_isolated/s

------2D
cp install_isolated/share/cartographer_ros/configuration_files/backpack_3d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua



-----3D
cp install_isolated/share/cartographer_ros/configuration_files/backpack_2d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua