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

    float frontLeftEngineThrottle = throttle - rpyPid.Pitch + rpyPid.Roll + rpyPid.Yaw;
    float frontRightEngineThrottle = throttle - rpyPid.Pitch - rpyPid.Roll - rpyPid.Yaw;
    float backLeftEngineThrottle = throttle + rpyPid.Pitch + rpyPid.Roll - rpyPid.Yaw;
    float backRightEngineThrottle = throttle + rpyPid.Pitch - rpyPid.Roll + rpyPid.Yaw;

    auto t_frontLeftEngineThrottle = frontLeftEngineThrottle , t_frontRightEngineThrottle = frontRightEngineThrottle;
    auto t_backLeftEngineThrottle = backLeftEngineThrottle , t_backRightEngineThrottle = backRightEngineThrottle;

    auto minThrottle = std::min({frontLeftEngineThrottle , frontRightEngineThrottle , backLeftEngineThrottle , backRightEngineThrottle});

    if(minThrottle < 1000)
    {
        frontLeftEngineThrottle =  (frontLeftEngineThrottle*1000) / minThrottle;
        frontRightEngineThrottle =  (frontRightEngineThrottle*1000) / minThrottle;
        backLeftEngineThrottle =  (backLeftEngineThrottle*1000) / minThrottle;
        backRightEngineThrottle =  (backRightEngineThrottle*1000) / minThrottle;
    }

#ifdef DEBUGOVERWIFI
    m_engineDebug = { frontLeftEngineThrottle , frontRightEngineThrottle , backLeftEngineThrottle ,backRightEngineThrottle };
#endif

    Serial.printf("Before = Front Left : %f , Front Right : %f , Back Left : %f , Back Right : %f\n" , 
    frontLeftEngineThrottle , frontRightEngineThrottle , backLeftEngineThrottle , backRightEngineThrottle);

    frontLeftEngineThrottle = map(frontLeftEngineThrottle , 1000 , 2000 , 0 , 180);
    frontRightEngineThrottle = map(frontRightEngineThrottle , 1000 , 2000 , 0 , 180);
    backLeftEngineThrottle = map(backLeftEngineThrottle , 1000 , 2000 , 0 , 180);
    backRightEngineThrottle = map(backRightEngineThrottle , 1000 , 2000 , 0 , 180);

    m_frontLeftEngine.write(frontLeftEngineThrottle);
    m_frontRightEngine.write(frontRightEngineThrottle);
    m_backLeftEngine.write(backLeftEngineThrottle);
    m_backRightEngine.write(backRightEngineThrottle);
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