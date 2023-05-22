#ifndef ALTITUDE_CONTROL_H
#define ALTITUDE_CONTROL_H

#include <DFRobot_URM09.h>
#include "Control.h"
#include <Communication/ReceiveCommand.h>
#include <AltitudeComputer/AltitudeComputer.h>

class AltitudePid
{
public:
    AltitudePid() = default;
    float pid(float desiredAltitude , float altitude , float velocity);

private:
    float m_kp{0.6f} , m_ki{0.65f} , m_kd{0.7f};
    float m_integralPrev{0.f} , m_errorPrev{0.f};
};


class AltitudeController : public Control
{
public:
    AltitudeController();
    float control(ReceiveCommand& command , 
    const std::pair<AltitudeComputer::Altitude , AltitudeComputer::Velocity>& altitudeAndVelocity);
    virtual void setFirstTime(float throttle) override;
    virtual void setFinishTime() override;
private:
    AltitudePid m_pid;
    float m_throttle{1000.f};
};



#endif