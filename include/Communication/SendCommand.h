#ifndef SEND_COMMAND_H
#define SEND_COMMAND_H

#include "ReceiveCommand.h"
#include <EasyTransfer.h>
#include <Imu/ImuDefinitions.h>

struct DebugValues
{
    float altitude{};
    float desired_altitude{};
    float p_gain_alt{};
    float P_alt{};
    float I_alt{};
    float D_alt{};
    int countOfLoop{};
};

struct SendInformations
{
    int altitude{};
    int satelliteCount{};
    Command status{};
    float roll{};
    float pitch{};
    float yaw{};
    float latitude{};
    float longtitude{};
    DebugValues debug;
};

class TransferData
{
public:
    TransferData();
    void transferData(const RPY& angles , int altitude , Command status , int countOfLoop);
private:
    EasyTransfer m_transfer;
    SendInformations m_infos;
    uint32_t m_lastSend{};
};



#endif