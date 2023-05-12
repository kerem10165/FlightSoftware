#ifndef RECEIVE_COMMAND_H
#define RECEIVE_COMMAND_H

enum class Command : int
{
    fly_joyistick = 0,
    set_altitude,
    set_pos,
    pos_hold,
    alt_hold,
    return_home,
    shut_down
};


struct ReceiveCommand
{
    Command command;
    float altitude;
    float lat;
    float lon;
};

#endif