#ifndef BAROMETER_SENSOR_H
#define BAROMETER_SENSOR_H

#include <BMP388_DEV.h>   
#include <utility>

class BarometerSensor
{
public:
    using Altitude = int;
    using ElapsedTime = uint32_t;  
public:
    BarometerSensor();
    std::pair<Altitude , ElapsedTime> readBarometer();
private:
    BMP388_DEV bmp388;    
    float m_temperature{0.f}, m_pressure{0.f}, m_altitude{0.f};
    uint32_t last_read{} , current_read{};

};


#endif