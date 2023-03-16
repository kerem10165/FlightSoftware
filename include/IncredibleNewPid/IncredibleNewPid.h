#ifndef PID_H
#define PID_H

#include <Imu/ImuDefinitions.h>

#ifdef DEBUGOVERWIFI
#include <array>
#include <Debug/DebugDefinitions.h>
#endif

class IncredibleNewPid
{
    enum class Choice
    {
        Roll = 0,
        Pitch = 1,
        Yaw = 2
    };
public:
    IncredibleNewPid(const RPY& kp , const RPY& ki , const RPY& kd , float integralLimit = 25.f);
    RPY getPid(const RPY &angels , const ImuData& rawImuData , const RPY &desiredAngles, float dt , float throttle);
private:
    float getPid(float angle , float rawImuData , float desiredAngle , float P , float I , float D , 
    float dt , float throttle , Choice choice , float& integralPrev , float& errorPrev);
    RPY m_kp , m_ki , m_kd;
    RPY m_integralPrev , m_errorPrev;
    float m_integralLimit;
};


#endif