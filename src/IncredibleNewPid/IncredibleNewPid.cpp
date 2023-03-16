#include <Arduino.h>
#include "IncredibleNewPid/IncredibleNewPid.h"


IncredibleNewPid::IncredibleNewPid(const RPY& kp , const RPY& ki , const RPY& kd , float integralLimit = 25.f)
    : m_kp{kp} , m_ki{ki} , m_kd{kd} , m_integralLimit{integralLimit}
{

}

RPY IncredibleNewPid::getPid(const RPY &angels , const ImuData& rawImuData , const RPY &desiredAngles, float dt , float throttle)
{
    auto rollPid = getPid(angels.Roll , rawImuData.GyroX , desiredAngles.Roll , m_kp.Roll , m_ki.Roll , m_kd.Roll,
    dt , throttle , Choice::Roll , m_integralPrev.Roll , m_errorPrev.Roll);

    auto pitchPid = getPid(angels.Pitch , rawImuData.GyroY , desiredAngles.Pitch , m_kp.Pitch , m_ki.Pitch , m_kd.Pitch,
    dt , throttle , Choice::Pitch , m_integralPrev.Pitch , m_errorPrev.Pitch);

    auto yawPid = getPid(rawImuData.GyroZ , rawImuData.GyroZ , desiredAngles.Yaw , m_kp.Yaw , m_ki.Yaw , m_kd.Yaw,
    dt , throttle , Choice::Yaw , m_integralPrev.Yaw , m_errorPrev.Yaw);

    return {rollPid , pitchPid , yawPid};
}

float IncredibleNewPid::getPid(float angle , float rawImuData , float desiredAngle , float P , float I , float D , 
    float dt , float throttle , Choice choice , float& integralPrev , float& errorPrev)
{
    float error = desiredAngle - angle;
    float integral = integralPrev + error*dt;
    if (throttle < 1225) {   //Don't let integrator build if throttle is too low
    integral = 0;
    }
    integral = constrain(integral, -m_integralLimit, m_integralLimit); //Saturate integrator to prevent unsafe buildup
    
    float derivative{0.f};
    if(choice != Choice::Yaw)
        derivative = rawImuData;
    else
        derivative = (error - errorPrev)/dt;

    float pid = 0.01*(P *error + I *integral - D *derivative);

    integralPrev = integral;
    errorPrev = error;

    return pid;
}