#ifndef ALTITUDE_COMPUTER_H
#define ALTITUDE_COMPUTER_H

#include "Barometer.h"
#include "Ultrasonic.h"
#include <utility>

class AltitudeComputer
{
public:
    using Altitude = int;
    using Velocity = float;
public:
    AltitudeComputer();
    std::pair<Altitude , Velocity> getAltitudeAndVerticalVelocity();
private:
    float getVelocity(uint32_t currentMeasurementTime , int currentAltitude , uint32_t lastMeasurementTime , int lastAltitude);
private:
    BarometerSensor barometer;
    UltrasonicSensor ultrasonic;
    int m_groundAltBarom{0};
    int m_lastAltitude{0};
    float m_velocity{0.f};
    uint32_t m_lastMeasurementTime{};
};







#endif