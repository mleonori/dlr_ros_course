clc; clear;

%% ROS bag parsing
% Get the ROS bag file name
if (ispc)
    rosbag_filename = ['~', filesep, 'ros/noetic/dlr_ros_course_ws/src/dlr_ros_course/dlr_ros_utils/bags/sensor_data.bag'];
else
    rosbag_filename = ['~/ros/noetic/dlr_course_ws/src/dlr_ros_course/dlr_ros_utils/bags/sensor_data.bag'];
end

% Open and parse ROS bag log file
bag = rosbag(rosbag_filename);

% Retrieve information from ROS bag
imu_data_1_sec = select(bag,'Time', [bag.StartTime bag.StartTime + 1],'Topic','/imu/data');
imu_data = select(bag,'Topic','/imu/data');
gps_data = select(bag,'Topic','/gps/data');

% Get available frames
tf = bag.AvailableFrames

% Get transformation between vessel base link and imu base link
vessel_base_link_T_imu_base_link = getTransform(bag,'vessel_base_link', 'imu_base_link');

% Read message as structure
imu_msgs = readMessages(imu_data,'DataFormat','struct');

% Get time vector
imu_time = cellfun(@(msg) double(msg.Header.Stamp.Sec) + (double(msg.Header.Stamp.Nsec) * 1e-9), imu_msgs);
imu_time = imu_time - imu_time(1);

% Extract all linear accelerations
linear_accelerations_x = cellfun(@(msg) double(msg.LinearAcceleration.X), imu_msgs);
linear_accelerations_y = cellfun(@(msg) double(msg.LinearAcceleration.Y), imu_msgs);
linear_accelerations_z = cellfun(@(msg) double(msg.LinearAcceleration.Z), imu_msgs);

%% Plot results
figure(1), clf;
plot(imu_time, linear_accelerations_x, "-.r");
hold on;
plot(imu_time, linear_accelerations_y, "-.g");
plot(imu_time, linear_accelerations_z, "-.b");
grid on;
legend("X", "Y", "Z");
title("Time vs. Linear acceleration");
xlabel("Time [s]");
ylabel("Acc [m/s^2]");
