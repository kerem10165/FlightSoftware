#include "Pid/Pid.h"
#include <Arduino.h>

Pid::Pid(const RPY& kp , const RPY& ki , const RPY& kd , float integralLimit)
    :m_kp{kp} , m_ki{ki} , m_kd{kd} , m_integralLimit{integralLimit}
{

}

RPY Pid::getPidValues(const RPY &eulers, const ImuData &rawImuData, const RPY &desiredValues , float throttle , float deltaTime)
{
    Serial.print("Roll ");
    float rollPid = getPidValue(desiredValues.roll , eulers.roll , rawImuData.GyroX , m_kp.roll , m_ki.roll , m_kd.roll,
        m_lastError.roll , m_lastIntegral.roll , throttle, deltaTime);

    Serial.print("Pitch ");
    float pitchPid = getPidValue(desiredValues.pitch , eulers.pitch , rawImuData.AccY , m_kp.pitch , m_ki.pitch , m_kd.pitch,
        m_lastError.pitch , m_lastIntegral.pitch , throttle, deltaTime);

    Serial.print("Yaw ");
    float yawPid = getPidValue(desiredValues.yaw , rawImuData.GyroZ , rawImuData.GyroZ ,  m_kp.yaw , m_ki.yaw , m_kd.yaw,
        m_lastError.yaw , m_lastIntegral.yaw , throttle, deltaTime);

    Serial.printf("Calculated pid values = RollPid : %f , PitchPid : %f , YawPid : %f\n" , rollPid , pitchPid , yawPid);

    return RPY{rollPid , pitchPid , yawPid};
}

float Pid::getPidValue(float desiredAngle , float angle , float rawAngle , float kp , float ki , float kd
    ,float& lastError , float& integralPrev , float throttle , float dt)
{
    float error = desiredAngle - angle;
    
    float integral = integralPrev + error*dt;
    if(throttle < 1120)
        integral = 0;
    integral = constrain(integral, -m_integralLimit, m_integralLimit);
    integralPrev = integral;
    
    float derivative = (error - lastError) / dt;
    lastError = error;

    Serial.printf(" Desired Angle : %f , Angle : %f , Integral : %f "
    ", IntegralPrev : %f , throttle : %f , dt : %f kp : %f , ki : %f , kd : %f\n" , 
    desiredAngle , angle , integral , integralPrev , throttle , dt , kp , ki ,kd);

    return kp*error + ki*integral - kd*derivative;
}

void Pid::resetLastPidValues()
{
    m_lastError = RPY{};
    m_lastIntegral = RPY{};
}