


# Build docker from DockerfileBuild
```
sudo docker build -t mleonori/ros-google-cartographer -f Dockerfile --build-arg ROS_DISTRO="noetic" .
```

```
sudo docker push mleonori/ros-google-cartographer
```

```
sudo docker pull mleonori/ros-google-cartographer
```

```
sudo docker run -it mleonori/ros-google-cartographer /bin/bash
```

In the docker container:
```
export ROS_MASTER_URI=http://your_master_ip_address:11311
export ROS_MASTER_URI=http://172.17.0.1:11311
export ROS_IP=172.17.0.2

2D Demo

Download the ROS bag in you computer
```
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
rosparam set /use_sim_time true
rosbag play ~/Downloads/cartographer_paper_deutsches_museum.bag --clock
```

In a new terminal open rviz


In the ros-google-cartographer docker execute the following commands:
```
#source /opt/cartographer_ros/setup.bash
roslaunch cartographer_ros backpack_2d.launch
```

3D

Download the ROS bag in you computer
```
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-13-54-42.bag
rosparam set /use_sim_time true
rosbag play ~/Downloads/b3-2016-04-05-13-54-42.bag --clock
```

In a new terminal open rviz

In the ros-google-cartographer docker execute the following commands:
```
#source /opt/cartographer_ros/setup.bash
roslaunch cartographer_ros backpack_2d.launch
```

Clone the dlr_ros_course repo
git clone https://github.com/mleonori/dlr_ros_course.git

apt install net-tools
ping 172.17.0.1
ifconfig
export ROS_MASTER_URI=http://172.17.0.1:11311
export ROS_IP=172.17.0.2
  
source /home/catkin_ws/devel/setup.bash
mon launch dlr_cart


To compile the ws in /home/catkin_ws folder:
```
catkin_make_isolated --install --use-ninja
```

To source the ws:
```
source /home/catkin_ws/devel_isolated/setup.bash
```
sudo docker cp dlr_vessel_2d.lua 212260287ba9:/home/catkin_ws/src/dlr_ros_course/dlr_mapping/dlr_cartographer/config