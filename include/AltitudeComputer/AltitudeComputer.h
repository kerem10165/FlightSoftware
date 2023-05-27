#ifndef ALTITUDE_COMPUTER_H
#define ALTITUDE_COMPUTER_H

#include "Barometer.h"
#include <utility>

#include <Imu/ImuDefinitions.h>

float pressureToAltitude(float pressure);
float altitudeToPressure(float altitude);

class AltitudeComputer
{
public:
    using Altitude = BarometerSensor::Altitude;
    using Pressure = BarometerSensor::Pressure;
    using ElapsedTime = BarometerSensor::ElapsedTime;
public:
    AltitudeComputer();
    std::tuple<Altitude , Pressure , ElapsedTime> getAltitudeAndPressure();
    float getGroundPressure() { return m_groundPressure; }
    float getGroundAltitudeFromPressure() { return m_groundAltitudeFromPressure; }
private:
    BarometerSensor barometer;
    float m_groundAlt{0.f};
    float actualPressure{0.f};
    float m_groundPressure{0.f};
    float m_groundAltitudeFromPressure{0.f};
};







#endif