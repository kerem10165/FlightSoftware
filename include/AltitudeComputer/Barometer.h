#ifndef BAROMETER_SENSOR_H
#define BAROMETER_SENSOR_H

#include <BMP388_DEV.h>   
#include <utility>
#include <tuple>

class BarometerSensor
{
public:
    using Altitude = float;
    using Pressure = float;
    using ElapsedTime = uint32_t;  
public:
    BarometerSensor();
    std::tuple<Altitude , Pressure , ElapsedTime> readBarometer();
private:
    BMP388_DEV bmp388;    
    float m_temperature{0.f}, m_pressure{0.f}, m_altitude{0.f};
    uint32_t last_read{} , current_read{};

};


#endif