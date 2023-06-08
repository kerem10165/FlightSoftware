#ifndef ALTITUDE_CONTROL_H
#define ALTITUDE_CONTROL_H

#include "Control.h"
#include <array>
#include <queue>

class ReceiveCommand;

class AltitudePid
{
public:
    AltitudePid() = default;
    AltitudePid(AltitudePid&& other);
    AltitudePid& operator=(AltitudePid&& other);
    
    float pid(float desiredAltitude , float altitude , float velocity);
    void setPidParams(float p , float i , float d);
private:
    float m_kp{0.f} , m_ki{0.f} , m_kd{0.f};
    float m_integralPrev{0.f} , m_errorPrev{0.f};
};

class AltitudeController : public Control
{
    friend class FlightControl;
public:
    AltitudeController();
    float control(float desiredAltitude ,float pressure , uint32_t elapsedTimeLastAltitudeMeasurement);
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