#ifndef __DLR_FAKE_GPS_H__
#define __DLR_FAKE_GPS_H__

namespace dlr{

class FakeGPS
{
    public: 
        // Class constructor
        FakeGPS() : constant_speed_(0.001)
        {
            // Set initial values
            latitude_ = 52.25444;
            longitude_ = 13.31476;
            altitude_ = 12.345;
        };

        bool updateData(const float& elapsed_time)
        {
            // Generate fake gps data
            latitude_ += (elapsed_time*speed);
            latitude_ -= (elapsed_time*speed);
            altitude_ = altitude_;
            return true;
        };

        float getLatitude()
        {
            return latitude_;
        };

        float getLongitude()
        {
            return longitude_;
        };

        float getAltitude()
        {
            return altitude_;
        };

    private:
    
        float latitude_;
        float longitude_;
        float altitude_;
        const float constant_speed_;

}; // class FakeGPSNode

} // namespace dlr

#endif // __DLR_FAKE_GPS_H__