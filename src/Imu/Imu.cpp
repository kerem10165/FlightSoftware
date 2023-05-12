#include <Arduino.h>
#include <Wire.h>
#include <Imu/Imu.h>

Imu::Imu(const ImuData& errorOfImu)
    :mpu6050{MPU6050{}} ,m_errorOfImu{errorOfImu}
{
    mpu6050.initialize();
    
    if (mpu6050.testConnection() == false) 
    {
      while(1) 
      {
        Serial.printf("Imu connection isn't succesfull\n");
      }
    }

    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}


ImuData Imu::getImuError() 
{
    int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
    ImuData errors{};

    size_t readCount{12'000};
    for(size_t i {0} ; i < readCount ; ++i)
    {
        mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
        float AccX  = AcX / ACCEL_SCALE_FACTOR;
        float AccY  = AcY / ACCEL_SCALE_FACTOR;
        float AccZ  = AcZ / ACCEL_SCALE_FACTOR;
        float GyroX = GyX / GYRO_SCALE_FACTOR;
        float GyroY = GyY / GYRO_SCALE_FACTOR;
        float GyroZ = GyZ / GYRO_SCALE_FACTOR;

        errors.AccX += AccX;
        errors.AccY += AccY;
        errors.AccZ += AccZ;
        errors.GyroX += GyroX;
        errors.GyroY += GyroY;
        errors.GyroZ += GyroZ;
    }

    errors.AccX/=readCount;
    errors.AccY/=readCount;
    (errors.AccZ/=readCount)-=1.f;
    errors.GyroX/=readCount;
    errors.GyroY/=readCount;
    errors.GyroZ/=readCount;

    return errors;
}

void Imu::printImuError(const ImuData& error) const
{
    Serial.printf("Errors AccX : %f AccY : %f AccZ : %f\nGyroX: %f GyroY : %f GyroZ : %f\n",
    error.AccX , error.AccY , error.AccZ , error.GyroX , error.GyroY , error.GyroZ);
}

const ImuData& Imu::getImuData()
{
    end = millis();
    if(end - last_read > 1)
    {
        mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
        last_read = end;
    }
    
    ImuData newImuData;

    newImuData.AccX = AcX / ACCEL_SCALE_FACTOR;
    newImuData.AccY = AcY / ACCEL_SCALE_FACTOR;
    newImuData.AccZ = AcZ / ACCEL_SCALE_FACTOR;

    newImuData.AccX-=m_errorOfImu.AccX;
    newImuData.AccY-=m_errorOfImu.AccY;
    newImuData.AccZ-=m_errorOfImu.AccZ;
    
    newImuData.AccX = (1.0 - B_accel)*m_mpuRawData.AccX + B_accel*newImuData.AccX;
    newImuData.AccY = (1.0 - B_accel)*m_mpuRawData.AccY + B_accel*newImuData.AccY;
    newImuData.AccZ = (1.0 - B_accel)*m_mpuRawData.AccZ + B_accel*newImuData.AccZ;
    
    newImuData.GyroX = GyX / GYRO_SCALE_FACTOR;
    newImuData.GyroY = GyY / GYRO_SCALE_FACTOR;
    newImuData.GyroZ = GyZ / GYRO_SCALE_FACTOR;

    newImuData.GyroX-=m_errorOfImu.GyroX;
    newImuData.GyroY-=m_errorOfImu.GyroY;
    newImuData.GyroZ-=m_errorOfImu.GyroZ;

    newImuData.GyroX = (1.0 - B_gyro)*m_mpuRawData.GyroX + B_gyro*newImuData.GyroX;
    newImuData.GyroY = (1.0 - B_gyro)*m_mpuRawData.GyroY + B_gyro*newImuData.GyroY;
    newImuData.GyroZ = (1.0 - B_gyro)*m_mpuRawData.GyroZ + B_gyro*newImuData.GyroZ;
    
    m_mpuRawData = newImuData;

    return m_mpuRawData;
}

RPY Imu::getRollPitchYaw(const ImuData& mpuRawData, float invSampleFreq)
{
    return getRollPitchYaw(mpuRawData.GyroX , -mpuRawData.GyroY , -mpuRawData.GyroZ , 
                            -mpuRawData.AccX , mpuRawData.AccY , mpuRawData.AccZ , invSampleFreq);
}

RPY Imu::getRollPitchYaw(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq)
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
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
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
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
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

    return RPY{atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951f , 
                -asinf(-2.0f * (q1*q3 - q0*q2))*57.29577951f , 
                -atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951f};
}