#ifndef PID_H
#define PID_H

#include <Imu/ImuDefinitions.h>

class AttitudeControlPid
{
    enum class Choice
    {
        Roll = 0,
        Pitch = 1,
        Yaw = 2
    };
public:
    AttitudeControlPid(const RPY& kp = RPY{} , const RPY& ki = RPY{} , const RPY& kd = RPY{} , float integralLimit = 27.5f);
    void setPidParams(const RPY& p , const RPY& i , const RPY& d);
    RPY getPid(const RPY &angels , const ImuData& rawImuData , const RPY &desiredAngles, float dt , float throttle);
private:
    float getPid(float angle , float rawImuData , float desiredAngle , float P , float I , float D , 
    float dt , float throttle , Choice choice , float& integralPrev , float& errorPrev);
    RPY m_kp , m_ki , m_kd;
    RPY m_integralPrev , m_errorPrev;
    float m_integralLimit;
};


#endif