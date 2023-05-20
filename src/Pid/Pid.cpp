#include <Arduino.h>
#include "Pid/Pid.h"

Pid::Pid(const RPY& kp , const RPY& ki , const RPY& kd , float integralLimit)
    : m_kp{kp} , m_ki{ki} , m_kd{kd} , m_integralLimit{integralLimit}
{
}

RPY Pid::getPid(const RPY &angels , const ImuData& rawImuData , const RPY &desiredAngles, float dt , float throttle)
{
    auto rollPid = getPid(angels.Roll , rawImuData.GyroX , desiredAngles.Roll , m_kp.Roll , m_ki.Roll , m_kd.Roll,
    dt , throttle , Choice::Roll , m_integralPrev.Roll , m_errorPrev.Roll);

    auto pitchPid = getPid(angels.Pitch , rawImuData.GyroY , desiredAngles.Pitch , m_kp.Pitch , m_ki.Pitch , m_kd.Pitch,
    dt , throttle , Choice::Pitch , m_integralPrev.Pitch , m_errorPrev.Pitch);

    float gyroZ = 0.f;
 
    if(rawImuData.GyroZ > -0.25 && rawImuData.GyroZ < 0.25)
        ;
    
    else
        gyroZ = rawImuData.GyroZ;
    
    auto yawPid = getPid(gyroZ , gyroZ , desiredAngles.Yaw , m_kp.Yaw , m_ki.Yaw , m_kd.Yaw,
    dt , throttle , Choice::Yaw , m_integralPrev.Yaw , m_errorPrev.Yaw);

    return {rollPid , pitchPid , yawPid};
}

float Pid::getPid(float angle , float rawImuData , float desiredAngle , float P , float I , float D , 
    float dt , float throttle , Choice choice , float& integralPrev , float& errorPrev)
{
    float error = desiredAngle - angle;
    float integral = integralPrev + error*dt;
    if (throttle < 1160 && choice != Choice::Yaw)
        integral = 0;
    
    if(throttle < 1160 && choice == Choice::Yaw)
        integral = 0;
    integral = constrain(integral, -m_integralLimit, m_integralLimit);
    
    float derivative{0.f};
    if(choice != Choice::Yaw)
        derivative = rawImuData;
    else
        derivative = (error - errorPrev)/dt;

    float pid = P *error + I *integral - D *derivative;

    integralPrev = integral;
    errorPrev = error;

    return pid;
}