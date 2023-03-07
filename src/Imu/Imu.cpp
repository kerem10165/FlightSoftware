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
        m_GyroError.Roll += gyroData.Roll;
        m_GyroError.Pitch += gyroData.Pitch;
        m_GyroError.Yaw += gyroData.Yaw;
        delay(1);
    }

    m_GyroError.Roll /= readCount;
    m_GyroError.Pitch /= readCount;
    m_GyroError.Yaw /= readCount;
}

RPY Imu::getGyro()
{
    auto gyroData = rawGyro();

    gyroData.Roll -= m_GyroError.Roll;
    gyroData.Pitch -= m_GyroError.Pitch;
    gyroData.Yaw -= m_GyroError.Yaw;

    return gyroData;
}

RPY Imu::rawGyro()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission(); 
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
    
    return RPY{GyroX/65.5f , GyroY/65.5f , GyroZ/65.5f};
}