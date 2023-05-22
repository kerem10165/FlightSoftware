#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <DFRobot_URM09.h>

class UltrasonicSensor
{
public:
    using Altitude = int;
    using ElapsedTime = uint32_t;
public:
    UltrasonicSensor();
    std::pair<Altitude , ElapsedTime> readUltrasonic();
private:
    DFRobot_URM09 URM09;
};





#endif