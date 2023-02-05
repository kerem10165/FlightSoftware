/*These code lines are taken from dRehmFlight software and have been modified for our own purposes.*/

#ifndef IMU_H_HEADER
#define IMU_H_HEADER

#include "ImuDefinitions.h"

#include <Wire.h>
#include "MPU6050.h"


struct ImuData
{
    float AccX = 0.f;
    float AccY = 0.f;
    float AccZ = 0.f;
    float GyroX = 0.f;
    float GyroY= 0.f;
    float GyroZ = 0.f;
};

struct RPY
{
    float roll = 0.f;
    float pitch = 0.f;
    float yaw = 0.f; 
};

void printImuData(const ImuData& x);
void printImuData(const RPY& x);


class Imu
{
public:
    Imu(int errorReadingCount = 12000);
    Imu(const ImuData& error);
    const ImuData& getImuData();
    const RPY& getRollPitchYaw(const ImuData& data, float invSampleFreq);
    void printImuError();
public:
    MPU6050 m_mpu6050{};
private:
    ImuData m_error;
    ImuData m_accelAndGyro;
    ImuData m_prevAccelAndGyro;
    RPY m_rollPitchYaw;

    const float filter_madgwick = 0.04f;
    const float filter_accel = 0.14f;     
    const float filter_gyro = 0.1f;

    float q0 = 1.0f; //Initialize quaternion for madgwick filter
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
private:
    ImuData getError(size_t errorReadingCount);
    void initilaizeImu();
    void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
};



#endif 