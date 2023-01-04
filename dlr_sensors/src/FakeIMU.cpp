#include "dlr_sensors/FakeIMU.h"

namespace dlr{
    
FakeIMU::FakeIMU()
{
    quat_.setIdentity();
};

bool FakeIMU::updateData(const float& elapsed_time)
{
    // Generate fake gps data
    // latitude_ += (elapsed_time * constant_speed_);
    // latitude_ -= (elapsed_time * constant_speed_);
    // altitude_ = altitude_;
    return true;
};

Eigen::Quaterniond FakeIMU::getQuaternion()
{
    return quat_;
}

Eigen::Vector3d FakeIMU::getEulerRPY()
{
    return Eigen::Vector3d();
}

} // namespace dlr