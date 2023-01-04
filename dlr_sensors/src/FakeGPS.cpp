#include "dlr_sensors/FakeGPS.h"
#include <iostream>

namespace dlr{
    
FakeGPS::FakeGPS() : constant_speed_(0.001f)
{
    // Set initial values
    latitude_ = 52.25444f;
    longitude_ = 13.31476f;
    altitude_ = 12.345f;
};

bool FakeGPS::updateData(const float& elapsed_time)
{
    // Generate fake gps data
    latitude_ += (elapsed_time * constant_speed_);
    longitude_ -= (elapsed_time * constant_speed_);
    altitude_ = altitude_;
    return true;
};

float FakeGPS::getLatitude()
{
    return latitude_;
};

float FakeGPS::getLongitude()
{
    return longitude_;
};

float FakeGPS::getAltitude()
{
    return altitude_;
};

} // namespace dlr