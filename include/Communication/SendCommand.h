#ifndef SEND_COMMAND_H
#define SEND_COMMAND_H

#include "ReceiveCommand.h"


struct SendInformations
{
    int altitude;
    int satelliteCount;
    Command status;
    float roll;
    float pitch;
    float yaw;
    float latitude;
    float longtitude;
};



#endif