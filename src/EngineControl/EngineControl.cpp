#include "EngineControl/EngineControl.h"

#include <algorithm>


EngineControl::EngineControl(int frontLeftEnginePin , int frontRightEnginePin , int backLeftEnginePin , int backRightEnginePin)
{
    m_frontLeftEngine.attach(frontLeftEnginePin , 1000 , 2000);
    m_frontRightEngine.attach(frontRightEnginePin , 1000 , 2000);
    m_backLeftEngine.attach(backLeftEnginePin , 1000 , 2000);
    m_backRightEngine.attach(backRightEnginePin , 1000 , 2000);
}

void EngineControl::driveEngine(float throttle , const RPY& rpyPid)
{
    if(throttle > 1750) 
        throttle = 1750;

    throttle -= 1000.f;
    throttle /= 1000.f;
    throttle = constrain(throttle , 0.f , 1.f); 

    float frontLeftEngineThrottle = throttle -rpyPid.Pitch + rpyPid.Roll + rpyPid.Yaw;
    float frontRightEngineThrottle = throttle -rpyPid.Pitch - rpyPid.Roll - rpyPid.Yaw;
    float backRightEngineThrottle = throttle +rpyPid.Pitch - rpyPid.Roll + rpyPid.Yaw;
    float backLeftEngineThrottle = throttle +rpyPid.Pitch + rpyPid.Roll - rpyPid.Yaw;

    // Serial.printf("Roll : %f , Pitch %f , Yaw : %f\n" , rpyPid.Roll *180 , rpyPid.Pitch * 180 , rpyPid.Yaw * 180);

    Serial.printf("Throttle : %f  ,Motor 1 : %d , Motor 2 : %d , Motor 3 : %d , Motor 4 : %d\n" , throttle , 
    static_cast<int>(frontLeftEngineThrottle *180) , static_cast<int>(frontRightEngineThrottle *180) , 
    static_cast<int>(backRightEngineThrottle *180) , static_cast<int>(backLeftEngineThrottle *180));
#ifdef DEBUGOVERWIFI
    m_engineDebug = { frontLeftEngineThrottle , frontRightEngineThrottle , backLeftEngineThrottle ,backRightEngineThrottle };
#endif

    m_frontLeftEngine.write(static_cast<int>(frontLeftEngineThrottle *180));
    m_frontRightEngine.write(static_cast<int>(frontRightEngineThrottle *180));
    m_backLeftEngine.write(static_cast<int>(backLeftEngineThrottle *180));
    m_backRightEngine.write(static_cast<int>(backRightEngineThrottle *180));
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