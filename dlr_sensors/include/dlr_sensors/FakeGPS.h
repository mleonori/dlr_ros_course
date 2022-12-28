#ifndef __DLR_FAKE_GPS_H__
#define __DLR_FAKE_GPS_H__

namespace dlr{

class FakeGPS
{
    public: 
        // Class constructor
        FakeGPS() : constant_speed_(0.001f)
        {
            // Set initial values
            latitude_ = 52.25444f;
            longitude_ = 13.31476f;
            altitude_ = 12.345f;
        };

        bool updateData(const float& elapsed_time);

        float getLatitude();

        float getLongitude();

        float getAltitude();
        
    private:
    
        float latitude_;
        float longitude_;
        float altitude_;
        const float constant_speed_;

}; // class FakeGPSNode

} // namespace dlr

#endif // __DLR_FAKE_GPS_H__