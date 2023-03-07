#ifndef RATE_PID_H
#define RATE_PID_H

#include <Imu/ImuDefinitions.h>

class RatePid
{
public:
    RatePid(const RPY& kp , const RPY& ki , const RPY& kd , float integralLimit = 200.f , float maxPidOutputLimit = 400.f);
    RPY getPidValues(const RPY& angels , const RPY &desiredAngles , float dt);
    void resetPidValues();
    void resetIntegralValues();
private:
    float getPidValue(float angle , float desiredAngle , float P , float I , float D,
     float& lastError , float& lastIntegral , float dt);
private:
    RPY m_kp;
    RPY m_ki;
    RPY m_kd;
    float m_integralLimit;
    float m_maxPidOutputLimit;
    RPY m_lastIntegral{};
    RPY m_lastError{};
};







#endif