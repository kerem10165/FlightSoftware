#ifndef RECEIVE_COMMAND_H
#define RECEIVE_COMMAND_H

#include <EasyTransfer.h>
#include <Settings/QuadSettings.h>

enum class Command : int
{
    fly_joyistick,
    set_altitude,
    set_pos,
    pos_hold,
    alt_hold,
    return_home,
    shut_down
};

enum class ReceiveType : int
{
    ReceiveData,
    ReceiveAltitudePidSetting,
    ReceiveAttitudePidSetting
};

struct ReceiveCommand
{
    Command command;
    float altitude;
    float lat;
    float lon;
};

struct ReceiveAltitudePidSetting
{
    float altitudeP;
    float altitudeI;
    float altitudeD;
};

struct ReceiveAttitudePidSetting
{
    float rollAndPitchP;
    float rollAndPitchI;
    float rollAndPitchD;
};

union ReceiveInformation
{
    ReceiveCommand receiveCommand;
    ReceiveAltitudePidSetting altitudePid;
    ReceiveAttitudePidSetting attitudePid;
};

struct ReceiveInformations
{
    char start[2];
    ReceiveType type{};
    ReceiveInformation data{};
    char end[2];
};


class ReceiveData
{
public:
    ReceiveData(QuadSettings * quadSettings , ReceiveCommand& receiveCommand);
    void receiveDatas();

private:
    EasyTransfer m_transfer;
    ReceiveInformations m_receivedData;
    QuadSettings* m_quadSettings;
    ReceiveCommand& m_receiveCommand;
};


#endif