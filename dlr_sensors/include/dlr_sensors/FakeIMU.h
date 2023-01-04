#ifndef __DLR_FAKE_IMU_H__
#define __DLR_FAKE_IMU_H__

#include <Eigen/Dense>

namespace dlr{

class FakeIMU
{
    public: 
        // Class constructor
        FakeIMU();

        bool updateData(const float& elapsed_time);

        Eigen::Quaterniond getQuaternion();

        Eigen::Vector3d getEulerRPY();
        
    private:
    
        Eigen::Quaterniond quat_;

}; // class FakeIMU

} // namespace dlr

#endif // __DLR_FAKE_IMU_H__