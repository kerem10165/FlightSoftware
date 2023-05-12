#ifndef ALTITUDE_CONTROL_H
#define ALTITUDE_CONTROL_H

#include <DFRobot_URM09.h>
#include "Control.h"
#include <Communication/ReceiveCommand.h>

class AltitudePid
{
public:
    AltitudePid() = default;
    float pid(float desiredAltitude , float altitude , float velocity);

private:
    float m_kp{0.5f} , m_ki{0.65f} , m_kd{0.7f};
    float m_integralPrev{0.f} , m_errorPrev{0.f};
};


class AltitudeController : public Control
{
public:
    AltitudeController();
    float control(ReceiveCommand& command);
    virtual void setFirstTime(float throttle) override;
    virtual void setFinishTime() override;
    std::pair<int16_t , uint32_t> readUltrasonic();
private:
    void setStartAltitude();
private:
    DFRobot_URM09 URM09;
    AltitudePid m_pid;
    float m_lastAltitude{0.f};
    float m_lastPid{0.f};
    float m_throttle{1000.f};
    float m_filteredAlt{0.f};
    float m_altitudeFilterCof{0.15};
};



#endif