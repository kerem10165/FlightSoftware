#ifndef PID_H_
#define PID_H_

#include "Imu/ImuDefinitions.h"

class Pid
{
public:
    Pid(const RPY& kp , const RPY& ki , const RPY& kd , float integralLimit);

    RPY getPidValues(const RPY& eulers , const ImuData& rawImuData , const RPY& desiredValues , float deltaTime);
    float getPidValue(float desiredAngle , float angle , float kp , float ki , float kd
    ,float& lastError , float& integralPrev , float dt);
private:
    RPY m_kp;
    RPY m_ki;
    RPY m_kd;
    float m_integralLimit;
    RPY m_lastError;
    RPY m_lastIntegral;
};




#endif