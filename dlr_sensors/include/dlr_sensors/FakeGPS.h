#ifndef __DLR_FAKE_GPS_H__
#define __DLR_FAKE_GPS_H__

namespace dlr{

class FakeGPS
{
    public: 
        // Class constructor
        FakeGPS();

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