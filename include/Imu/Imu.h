#ifndef IMU_H_
#define IMU_H_

#include <cstddef>
#include <utility>
#include "ImuDefinitions.h"



class Imu
{
    using GYRO = RPY;
    using ACCELOMETER = RPY;
    using RawImuData = std::pair<GYRO , ACCELOMETER>;

    using State = float;
    using Uncertainty = float;
public:
    Imu(size_t readCount);
    RPY getGyro();
    RPY getRollPitchGyroZ();
private:
    RawImuData rawGyro();
    std::pair<State,Uncertainty> kalman(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
    RPY getRawRollPitchGyroZ();
private:
    RPY m_GyroError{};
    float m_kalmanAngleRoll=0.f;
    float m_kalmanUncertaintyAngleRoll=2.f*2;
    float m_kalmanAnglePitch=0.f;
    float m_kalmanUncertaintyAnglePitch=2.f*2;

    float m_rollError{0.f};
    float m_pitchError{0.f};
};






#endif