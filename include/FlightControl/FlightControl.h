#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <Receiver/Receiver.h>
#include <Imu/Imu.h>
#include <EngineControl/EngineControl.h>
#include <Pid/Pid.h>
#include <Communication/ReceiveCommand.h>
#include <FlightControl/AltitudeController.h>

class FlightControl
{
public:
    FlightControl();
    void armEngine();
    void control(ReceiveCommand& command , const ImuData& rawImuData , const RPY& angles , Receiver& receiver , float dt);
private:
    void controlWJoyistick(const ImuData& rawImuData , const RPY& angles , Receiver& receiver , const ReceiverInput& input , float dt);
    void driveEngines(float throttle , const ReceiverInput& input , const RPY& quadPid);
    bool updateLastThrottleAndControlOwner(ReceiveCommand& command , Control& controller , float throttle);
private:
    EngineControl* m_engines;
    AltitudeController alitutdeControl;
    const RPY m_maxValues{30.f,30.f,150.f};
    const RPY m_kp{0.305,0.305,1.4f};
    const RPY m_ki{0.475,0.475,0.00105f};
    const RPY m_kd{0.105,0.105,0.000075};
    Pid m_pid_quad{m_kp,m_ki,m_kd};
    float m_last_throttle = 1000.f;
    bool m_throttle_has_setted{false};
};



#endif