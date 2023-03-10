#ifndef RATE_PID_H
#define RATE_PID_H

#include <Imu/ImuDefinitions.h>

#ifdef DEBUGOVERWIFI
#include <array>
#include <Debug/DebugDefinitions.h>
#endif

class Pid
{
    enum class DebugChoice
    {
        Roll = 0,
        Pitch = 1,
        Yaw = 2
    };
public:
    Pid(const RPY& kp , const RPY& ki , const RPY& kd , float integralLimit = 100.f ,
     float maxPidOutputLimit = 150.f , float derrivateLimit = 25.f);
    RPY getPidValues(const RPY& angels , const RPY &desiredAngles , float dt);
    void resetPidValues();
    void resetIntegralValues();
private:
    float getPidValue(float angle , float desiredAngle , float P , float I , float D,
     float& lastError , float& lastIntegral , float dt , DebugChoice choice);
private:
    RPY m_kp;
    RPY m_ki;
    RPY m_kd;
    float m_integralLimit;
    float m_maxPidOutputLimit;
    float m_derivativeLimit;
    RPY m_lastIntegral{};
    RPY m_lastError{};
#ifdef DEBUGOVERWIFI
public:
    std::array<PidDebugDefinitions,3> m_pidDebug;
#endif
};






#endif