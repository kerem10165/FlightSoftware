#ifndef ALTITUDE_CONTROL_H
#define ALTITUDE_CONTROL_H

#include <DFRobot_URM09.h>
#include "Control.h"
#include <Communication/ReceiveCommand.h>
#include <array>
#include <queue>

class AltitudePid
{
public:
    AltitudePid() = default;
    float pid(float desiredAltitude , float altitude , float velocity);
private:
    float m_kp{0.2f} , m_ki{0.6f} , m_kd{0.4f};
    float m_integralPrev{0.f} , m_errorPrev{0.f};
};

class AltitudeController : public Control
{
    friend class FlightControl;
public:
    AltitudeController();
    float control(ReceiveCommand& command ,float pressure , uint32_t elapsedTimeLastAltitudeMeasurement);
    void resetValues(float throttle , float altitude);
    virtual void setFirstTime(float throttle , float Altitude) override;
    virtual void setFinishTime() override;
private:
    AltitudePid m_pid;
    float m_throttle{1000.f};
    std::queue<float> m_verticalSpeeds;
    float m_lastAltitude{0.f};
    float m_accumuleteVerticalSpeed{0.f};
    int m_countOfReset{0};
    static const int verticalReadCount; 
};




#endif