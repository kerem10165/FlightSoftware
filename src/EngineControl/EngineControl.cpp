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
    float frontLeftEngineThrottle = throttle - rpyPid.pitch + rpyPid.roll + rpyPid.yaw;
    float frontRightEngineThrottle = throttle - rpyPid.pitch - rpyPid.roll - rpyPid.yaw;
    float backLeftEngineThrottle = throttle + rpyPid.pitch + rpyPid.roll - rpyPid.yaw;
    float backRightEngineThrottle = throttle + rpyPid.pitch - rpyPid.roll + rpyPid.yaw;

    auto maxThrottle = std::max({frontLeftEngineThrottle , frontRightEngineThrottle , backLeftEngineThrottle , backRightEngineThrottle});

    if(maxThrottle >= 2000)
    {
        frontLeftEngineThrottle = (frontLeftEngineThrottle / maxThrottle) * 2000;
        frontRightEngineThrottle = (frontRightEngineThrottle / maxThrottle) * 2000;
        backLeftEngineThrottle = (backLeftEngineThrottle / maxThrottle) * 2000;
        backRightEngineThrottle = (backRightEngineThrottle / maxThrottle) * 2000;
    }

    frontLeftEngineThrottle = map(frontLeftEngineThrottle , 1000 , 2000 , 0 , 160);
    frontRightEngineThrottle = map(frontRightEngineThrottle , 1000 , 2000 , 0 , 160);
    backLeftEngineThrottle = map(backLeftEngineThrottle , 1000 , 2000 , 0 , 160);
    backRightEngineThrottle = map(backRightEngineThrottle , 1000 , 2000 , 0 , 160);

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