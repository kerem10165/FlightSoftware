#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <Receiver/Receiver.h>
#include <Imu/Imu.h>
#include <EngineControl/EngineControl.h>
#include <Pid/AttitudeControlPid.h>
#include <AltitudeComputer/AltitudeComputer.h>
#include <FlightControl/AltitudeController.h>


class ReceiveCommand;

class FlightControl
{
public:
    FlightControl();
    void armEngine();
    void control(ReceiveCommand& command, const ImuData& rawImuData , const RPY& angles,
    uint32_t elapsedTimeLastAltitudeMeasurement ,float groundAltitude, float pressure, Receiver& receiver , float dt);
    void setAttitudeControlPid(const RPY& p , const RPY& i , const RPY& d);
    void setAltitudeControlPid(float p , float i , float d);
private:
    void controlWJoyistick(const ImuData& rawImuData , const RPY& angles , Receiver& receiver , const ReceiverInput& input , float dt);
    void driveEngines(float throttle , const ReceiverInput& input , const RPY& quadPid);
    bool updateLastThrottleAndControlOwner(ReceiveCommand& command , Control& controller , float throttle , float altitude);
private:
    EngineControl* m_engines;
    AltitudeController m_alitutdeControl;
    const RPY m_maxValues{30.f,30.f,150.f};
    AttitudeControlPid m_pid_quad;
    float m_last_throttle = 1000.f;
    bool m_throttle_has_setted{false};
};



#endif