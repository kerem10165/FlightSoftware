#ifndef IMU_H_
#define IMU_H_

#include "MPU6050.h"
#include "ImuDefinitions.h"

class Imu
{
public:
    Imu(const ImuData& errorOfImu);
    ImuData setImuError();
    void printImuError();
    //this function have to be called before call getRollPitchYaw function
    const ImuData& getImuData();
    RPY getRollPitchYaw(const ImuData& mpuRawData, float invSampleFreq);
private:
    RPY getRollPitchYaw(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
    MPU6050 mpu6050;
    ImuData m_errorOfImu;
    ImuData m_mpuRawData{};
    float B_madgwick = 0.04;
    float B_accel = 0.14f;
    float B_gyro = 0.09f;

    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
};



#endif