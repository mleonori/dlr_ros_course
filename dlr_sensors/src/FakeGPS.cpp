#include "dlr_sensors/FakeGPS.h"

namespace dlr{
    
FakeGPS::FakeGPS() : constant_speed_(0.001)
{
    // Set initial values
    latitude_ = 52.25444;
    longitude_ = 13.31476;
    altitude_ = 12.345;
};

bool FakeGPS::updateData(const float& elapsed_time)
{
    // Generate fake gps data
    latitude_ += (elapsed_time * constant_speed_);
    latitude_ -= (elapsed_time * constant_speed_);
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