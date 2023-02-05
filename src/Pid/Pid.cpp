#include "Pid/Pid.h"
#include <Arduino.h>

Pid::Pid(const RPY& kp , const RPY& ki , const RPY& kd , float integralLimit)
    :m_kp{kp} , m_ki{ki} , m_kd{kd} , m_integralLimit{integralLimit}
{

}

RPY Pid::getPidValues(const RPY &eulers, const ImuData &rawImuData, const RPY &desiredValues , float deltaTime)
{
    float rollPid = getPidValue(desiredValues.roll , eulers.roll , m_kp.roll , m_ki.roll , m_kd.roll,
        m_lastError.roll , m_lastIntegral.roll , deltaTime);

    float pitchPid = getPidValue(desiredValues.pitch , eulers.pitch , m_kp.pitch , m_ki.pitch , m_kd.pitch,
        m_lastError.pitch , m_lastIntegral.pitch , deltaTime);

    float yawPid = getPidValue(desiredValues.yaw , rawImuData.GyroZ , m_kp.yaw , m_ki.yaw , m_kd.yaw,
        m_lastError.yaw , m_lastIntegral.yaw , deltaTime);

    return RPY{rollPid , pitchPid , yawPid};
}

float Pid::getPidValue(float desiredAngle , float angle , float kp , float ki , float kd
    ,float& lastError , float& integralPrev , float dt)
{
    float error = desiredAngle - angle;
    
    float integral = integralPrev + error*dt;
    integral = constrain(integral, -m_integralLimit, m_integralLimit);
    integralPrev = integral;
    
    float derivative = (error - lastError) / dt;
    lastError = error;

    return kp*error + ki*integral - kd*derivative;
}
