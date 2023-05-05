#include "EngineControl/EngineControl.h"


EngineControl::EngineControl(int frontLeftEnginePin , int frontRightEnginePin , int backLeftEnginePin , int backRightEnginePin)
{
    m_frontLeftEngine.attach(frontLeftEnginePin , 1000 , 2000);
    m_frontRightEngine.attach(frontRightEnginePin , 1000 , 2000);
    m_backLeftEngine.attach(backLeftEnginePin , 1000 , 2000);
    m_backRightEngine.attach(backRightEnginePin , 1000 , 2000);
}

void EngineControl::driveEngines(float throttle , const RPY& rpyPid)
{
    if(throttle > 1750) 
        throttle = 1750;

    throttle -= 1000.f;
    throttle = (throttle / 1000.f)*180;
    throttle = constrain(throttle , 0.f , 180.f); 

    float frontLeftEngineThrottle = throttle -rpyPid.Pitch + rpyPid.Roll + rpyPid.Yaw;
    float frontRightEngineThrottle = throttle -rpyPid.Pitch - rpyPid.Roll - rpyPid.Yaw;
    float backRightEngineThrottle = throttle +rpyPid.Pitch - rpyPid.Roll + rpyPid.Yaw;
    float backLeftEngineThrottle = throttle +rpyPid.Pitch + rpyPid.Roll - rpyPid.Yaw;

    m_frontLeftEngine.write(frontLeftEngineThrottle);
    m_frontRightEngine.write(frontRightEngineThrottle);
    m_backRightEngine.write(backRightEngineThrottle);
    m_backLeftEngine.write(backLeftEngineThrottle);
}

void EngineControl::failSafe()
{
    m_frontLeftEngine.write(0);
    m_frontRightEngine.write(0);
    m_backLeftEngine.write(0);
    m_backRightEngine.write(0);
}

void EngineControl::startupEngines()
{
    m_frontLeftEngine.write(0);
    m_frontRightEngine.write(0);
    m_backLeftEngine.write(0);
    m_backRightEngine.write(0);
}