/*These code lines are taken from dRehmFlight software and have been modified for our own purposes.*/

#ifndef RECEIVER_H
#define RECEIVER_H

#include <PulsePosition.h>
#include "Imu/ImuDefinitions.h"


struct ReceiverInput
{
    float roll{1500.f};
    float pitch{1500.f};
    float throttle{1000.f};
    float yaw{1500.f};
    float switch1{1000.f};
    float switch2{1000.f};
};

struct Receiver
{
    Receiver(int pin);
    
    ReceiverInput* getCommand();
    RPY scaleRollPitchYawCommand(const ReceiverInput& inputs,const RPY& maxValues);
    void printReceiver();
    
    static ReceiverInput m_inputs;
    ReceiverInput m_prevInput{};
    static int m_pin;
    static unsigned long lastReadTime;
};



#endif