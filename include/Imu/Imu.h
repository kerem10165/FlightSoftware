#ifndef IMU_H_
#define IMU_H_

#include <cstddef>
#include "ImuDefinitions.h"

class Imu
{
public:
    Imu(size_t readCount);
    RPY getGyro();
private:
    RPY rawGyro();
private:
    RPY m_GyroError{};
};






#endif