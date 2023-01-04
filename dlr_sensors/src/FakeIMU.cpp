#include "dlr_sensors/FakeIMU.h"
#include <iostream>

namespace dlr{
    
FakeIMU::FakeIMU()
{
    quat_.setIdentity();
    gravity_ << 0.0, 0.0, 9.81; 
    t_ = 0;
};

bool FakeIMU::updateData(const float& elapsed_time)
{
    t_ += elapsed_time;

    // From quaternion to Euler angles
    auto euler_angles = quat_.toRotationMatrix().eulerAngles(0, 1, 2);
    
    // Generate fake imu data
    angular_velocities_(0) = sin(t_*0.8);
    angular_velocities_(1) = sin(t_*0.00);
    angular_velocities_(2) = sin(t_*0.00);
    euler_angles += (elapsed_time*angular_velocities_);
    // std::cout << "Euler Angles: " << euler_angles.transpose() << std::endl;

    // From Euler angles to quaternion
    quat_ = fromEulerAngles2Quat(euler_angles);
    // std::cout << "Quaterions: " <<  quat_.coeffs().transpose() << std::endl;

    // Compute fake linear accelerations
    linear_accelerations_ << quat_.toRotationMatrix() * gravity_;
    // std::cout << "Linear accelerations: " <<  linear_accelerations_.transpose() << std::endl;

    return true;
};

Eigen::Quaterniond FakeIMU::getQuaternion()
{
    return quat_;
}

Eigen::Vector3d FakeIMU::getEulerRPY()
{
    return fromQuat2EulerAngles(quat_);
}

Eigen::Vector3d FakeIMU::fromQuat2EulerAngles(const Eigen::Quaterniond& quat)
{
    return quat.toRotationMatrix().eulerAngles(0, 1, 2);
}

Eigen::Vector3d FakeIMU::getAngularVelocities()
{
    return angular_velocities_;
}

Eigen::Vector3d FakeIMU::getLinearAccelerations()
{
    return linear_accelerations_;
}

Eigen::Quaterniond FakeIMU::fromEulerAngles2Quat(const Eigen::Vector3d& euler_angles)
{
    Eigen::Quaterniond quat;
    quat =  Eigen::AngleAxisd(euler_angles(0), Eigen::Vector3d::UnitX()) * 
            Eigen::AngleAxisd(euler_angles(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler_angles(2), Eigen::Vector3d::UnitZ());
    return quat;
}

} // namespace dlr