#ifndef __DLR_FAKE_IMU_H__
#define __DLR_FAKE_IMU_H__

#include <Eigen/Dense>

namespace dlr{

class FakeIMU
{
    public:
        FakeIMU();

        bool updateData(const float& elapsed_time);

        Eigen::Quaterniond getQuaternion();
        Eigen::Vector3d getEulerRPY();
        Eigen::Vector3d getAngularVelocities();
        Eigen::Vector3d getLinearAccelerations();
        
    private:
    
        Eigen::Quaterniond quat_;
        Eigen::Vector3d angular_velocities_;
        Eigen::Vector3d linear_accelerations_;
        Eigen::Vector3d gravity_;
        float t_;

        Eigen::Vector3d fromQuat2EulerAngles(const Eigen::Quaterniond& quat);
        Eigen::Quaterniond fromEulerAngles2Quat(const Eigen::Vector3d& euler_angles);

}; // class FakeIMU

} // namespace dlr

#endif // __DLR_FAKE_IMU_H__