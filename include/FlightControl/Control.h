#ifndef CONTROL_H
#define CONTROL_H


class Control
{
public:
    virtual void setFirstTime(float throttle , float Altitude) = 0;
    virtual void setFinishTime() = 0;
};


#endif