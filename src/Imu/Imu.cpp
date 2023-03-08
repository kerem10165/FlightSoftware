#include <Arduino.h>
#include <Wire.h>
#include <Imu/Imu.h>

Imu::Imu(size_t readCount)
{
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68); 
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    for(size_t i{} ; i < readCount ; ++i)
    {
        auto gyroData = rawGyro();
        m_GyroError.Roll += gyroData.first.Roll;
        m_GyroError.Pitch += gyroData.first.Pitch;
        m_GyroError.Yaw += gyroData.first.Yaw;
        delay(1);
    }

    m_GyroError.Roll /= readCount;
    m_GyroError.Pitch /= readCount;
    m_GyroError.Yaw /= readCount;

    for(size_t i{} ; i < readCount/4 ; ++i)
    {
        auto rollPitchGyro = getRawRollPitchGyroZ();
        m_rollError += rollPitchGyro.Roll;
        m_pitchError += rollPitchGyro.Pitch;
    }

    m_rollError /= (readCount/4);
    m_pitchError /= (readCount/4);
}

Imu::GYRO Imu::getGyro()
{
    auto gyroData = rawGyro();

    gyroData.first.Roll -= m_GyroError.Roll;
    gyroData.first.Pitch -= m_GyroError.Pitch;
    gyroData.first.Yaw -= m_GyroError.Yaw;

    return gyroData.first;
}

RPY Imu::getRollPitchGyroZ()
{
    auto rawRPGz = getRawRollPitchGyroZ();
    rawRPGz.Roll -= m_rollError;
    rawRPGz.Pitch -= m_pitchError;
    
    return rawRPGz;
}

Imu::RawImuData Imu::rawGyro()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(); 
    Wire.requestFrom(0x68,6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(); 
    Wire.requestFrom(0x68,6);
    
    int16_t GyroX=Wire.read()<<8 | Wire.read();
    int16_t GyroY=Wire.read()<<8 | Wire.read();
    int16_t GyroZ=Wire.read()<<8 | Wire.read();
    
    return {GYRO{GyroX/65.5f , GyroY/65.5f , GyroZ/65.5f} , ACCELOMETER{(float)AccXLSB/4096 , (float)AccYLSB/4096, (float)AccZLSB/4096}};
}

std::pair<Imu::State,Imu::Uncertainty> Imu::kalman(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement) 
{
    kalmanState= kalmanState+0.004*kalmanInput;
    kalmanUncertainty= kalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float kalmanGain= kalmanUncertainty * 1/(1*kalmanUncertainty + 3 * 3);
    kalmanState= kalmanState+kalmanGain * (kalmanMeasurement-kalmanState);
    kalmanUncertainty=(1-kalmanGain) * kalmanUncertainty;

    return {kalmanState , kalmanUncertainty};
}

RPY Imu::getRawRollPitchGyroZ()
{
    auto gyroData = rawGyro();

    gyroData.first.Roll -= m_GyroError.Roll;
    gyroData.first.Pitch -= m_GyroError.Pitch;
    gyroData.first.Yaw -= m_GyroError.Yaw;

    auto angleRoll = atan(gyroData.second.Pitch/sqrt(gyroData.second.Roll *gyroData.second.Roll+ gyroData.second.Yaw*gyroData.second.Yaw))*1/(3.142/180);
    auto rollKalman = kalman(m_kalmanAngleRoll, m_kalmanUncertaintyAngleRoll, gyroData.first.Roll, angleRoll);
    m_kalmanAngleRoll = rollKalman.first;
    m_kalmanUncertaintyAngleRoll = rollKalman.second;

    auto anglePitch = -atan(gyroData.second.Roll/sqrt(gyroData.second.Pitch*gyroData.second.Pitch+gyroData.second.Yaw*gyroData.second.Yaw))*1/(3.142/180);
    auto pitchKalman = kalman(m_kalmanAnglePitch, m_kalmanUncertaintyAnglePitch, gyroData.first.Pitch, anglePitch);
    m_kalmanAnglePitch = pitchKalman.first;
    m_kalmanUncertaintyAnglePitch = pitchKalman.second;

    return {m_kalmanAngleRoll , m_kalmanAnglePitch , gyroData.first.Yaw};
}

