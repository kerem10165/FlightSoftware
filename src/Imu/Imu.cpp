#include <Arduino.h>
#include "Imu/Imu.h"

Imu::Imu(int errorReadingCount)
{
    initilaizeImu();
    delay(100);
    m_error = getError(errorReadingCount);
}

Imu::Imu(const ImuData& error)
    :m_error{error}
{
    initilaizeImu();
}

const ImuData& Imu::getImuData()
{
    int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
    m_mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    m_accelAndGyro.AccX = AcX / ACCEL_SCALE_FACTOR - m_error.AccX;
    m_accelAndGyro.AccY = AcY / ACCEL_SCALE_FACTOR - m_error.AccY;
    m_accelAndGyro.AccZ = AcZ / ACCEL_SCALE_FACTOR - m_error.AccZ;

    
    //low pass filter
    m_accelAndGyro.AccX = (1.0 - filter_accel) * m_prevAccelAndGyro.AccX + filter_accel * m_accelAndGyro.AccX;
    m_accelAndGyro.AccY = (1.0 - filter_accel) * m_prevAccelAndGyro.AccY + filter_accel * m_accelAndGyro.AccY;
    m_accelAndGyro.AccZ = (1.0 - filter_accel) * m_prevAccelAndGyro.AccZ + filter_accel * m_accelAndGyro.AccZ;
    
    m_prevAccelAndGyro.AccX = m_accelAndGyro.AccX;
    m_prevAccelAndGyro.AccY = m_accelAndGyro.AccY; 
    m_prevAccelAndGyro.AccZ = m_accelAndGyro.AccZ;

    m_accelAndGyro.GyroX = GyX / GYRO_SCALE_FACTOR - m_error.GyroX; //deg/sec
    m_accelAndGyro.GyroY = GyY / GYRO_SCALE_FACTOR - m_error.GyroY;
    m_accelAndGyro.GyroZ = GyZ / GYRO_SCALE_FACTOR - m_error.GyroZ;

    //LP filter gyro data
    m_accelAndGyro.GyroY = (1.0 - filter_gyro)* m_prevAccelAndGyro.GyroY + filter_gyro * m_accelAndGyro.GyroY;
    m_accelAndGyro.GyroZ = (1.0 - filter_gyro)* m_prevAccelAndGyro.GyroZ + filter_gyro * m_accelAndGyro.GyroZ;
    m_accelAndGyro.GyroX = (1.0 - filter_gyro)* m_prevAccelAndGyro.GyroX + filter_gyro * m_accelAndGyro.GyroX;
    
    m_prevAccelAndGyro.GyroY = m_accelAndGyro.GyroY;
    m_prevAccelAndGyro.GyroZ = m_accelAndGyro.GyroZ;
    m_prevAccelAndGyro.GyroX = m_accelAndGyro.GyroX;

    return m_accelAndGyro;
}

void Imu::initilaizeImu()
{
    Wire.begin();
    Wire.setClock(1000000);

    m_mpu6050.initialize();

    if (m_mpu6050.testConnection() == false) 
    {
        while(1) 
        {
            Serial.println("MPU6050 initialization unsuccessful");
        }
    }

    m_mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    m_mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}


ImuData Imu::getError(size_t errorReadingCount)
{
    int16_t AcX{},AcY{},AcZ{},GyX{},GyY{},GyZ{};
    
    float AccErrorX {} , AccErrorY{} , AccErrorZ{} , GyroErrorX{} , GyroErrorY{} , GyroErrorZ{};

    float AccX{} , AccY{} , AccZ{} , GyroX{}, GyroY{} , GyroZ{};

    m_mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    for(size_t i{} ; i < errorReadingCount; ++i)
    {
        m_mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

        AccX  = AcX / ACCEL_SCALE_FACTOR;
        AccY  = AcY / ACCEL_SCALE_FACTOR;
        AccZ  = AcZ / ACCEL_SCALE_FACTOR;
        GyroX = GyX / GYRO_SCALE_FACTOR;
        GyroY = GyY / GYRO_SCALE_FACTOR;
        GyroZ = GyZ / GYRO_SCALE_FACTOR;

        AccErrorX  = AccErrorX + AccX;
        AccErrorY  = AccErrorY + AccY;
        AccErrorZ  = AccErrorZ + AccZ;
        GyroErrorX = GyroErrorX + GyroX;
        GyroErrorY = GyroErrorY + GyroY;
        GyroErrorZ = GyroErrorZ + GyroZ;
    }

    return ImuData{AccErrorX /  errorReadingCount , AccErrorY /  errorReadingCount , AccErrorZ / errorReadingCount - 1.f,
                    GyroErrorX / errorReadingCount , GyroErrorY / errorReadingCount , GyroErrorZ / errorReadingCount};
}

const RPY& Imu::getRollPitchYaw(const ImuData& data, float invSampleFreq)
{
    Madgwick6DOF(data.GyroX , -data.GyroY, -data.GyroZ , -data.AccX , data.AccY , data.AccZ , invSampleFreq);

    return m_rollPitchYaw;
}

void Imu::Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) 
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    //Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    //Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {
        //Normalise accelerometer measurement
        recipNorm = 1.f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        //Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = 1.f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        //Apply feedback step
        qDot1 -= filter_madgwick * s0;
        qDot2 -= filter_madgwick * s1;
        qDot3 -= filter_madgwick * s2;
        qDot4 -= filter_madgwick * s3;
    }

    //Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    //Normalise quaternion
    recipNorm = 1.f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //Compute angles
    m_rollPitchYaw.roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
    m_rollPitchYaw.pitch = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
    m_rollPitchYaw.yaw = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void Imu::printImuError()
{
    Serial.printf("Errors!!!!!!!!!!!!!!!!!! Accel x : %f - Accel y : %f - Accel z : %f - Gyro x : %f - Gyro y : %f - Gyro z : %f\n",
    m_error.AccX , m_error.AccY , m_error.AccZ , m_error.GyroX , m_error.GyroY , m_error.GyroZ);
}


void printImuData(const ImuData& x)
{
    Serial.printf("Accel x : %f - Accel y : %f - Accel z : %f - Gyro x : %f - Gyro y : %f - Gyro z : %f\n",
    x.AccX , x.AccY , x.AccZ , x.GyroX , x.GyroY , x.GyroZ);
}

void printImuData(const RPY& x)
{
    Serial.printf("Roll = %f - Pitch = %f - Yaw = %f\n" , x.roll , x.pitch , x.yaw);
}