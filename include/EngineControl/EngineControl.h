#ifndef ENGINE_CONTROL_H
#define ENGINE_CONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include "Imu/ImuDefinitions.h"

class EngineControl
{
public:
    EngineControl(int frontLeftEnginePin , int frontRightEnginePin , int backLeftEnginePin , int backRightEnginePin);
    void driveEngine(float throttle , const RPY& rpyPid);
    void failSafe();
    void startupEngines();
private:
    Servo m_frontLeftEngine;
    Servo m_frontRightEngine;
    Servo m_backLeftEngine;
    Servo m_backRightEngine;
};





#endif