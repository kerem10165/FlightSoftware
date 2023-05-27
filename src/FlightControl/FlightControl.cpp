#include "FlightControl/FlightControl.h"
#include <FlightControl/Control.h>
#include <AltitudeComputer/AltitudeComputer.h>
FlightControl::FlightControl()
{

}

void FlightControl::armEngine()
{
    m_engines = new EngineControl{12,14,11,13};
}

void FlightControl::control(ReceiveCommand& command, const ImuData& rawImuData , const RPY& angles,
    uint32_t elapsedTimeLastAltitudeMeasurement ,float groundAltitude, float pressure, Receiver& receiver , float dt)
{
    if(auto input = receiver.getCommand())
    {
        if(command.command == Command::fly_joyistick)
        {
            controlWJoyistick(rawImuData , angles , receiver , *input , dt);
            m_throttle_has_setted = false;
        }
        else if(command.command == Command::set_altitude)
        {
            auto altitude = pressureToAltitude(pressure);
            if(updateLastThrottleAndControlOwner(command , alitutdeControl , input->throttle , altitude))
                return;
            
            if(input->switch1 > 1500)
            {
                auto throttle = alitutdeControl.control(command , pressure , elapsedTimeLastAltitudeMeasurement);
                auto scaledRollPitchYawInput = receiver.scaleRollPitchYawCommand(*input ,m_maxValues);
                auto quadPid = m_pid_quad.getPid(angles , rawImuData , scaledRollPitchYawInput , dt , throttle);
                driveEngines(throttle , *input , quadPid);
            }

            else
            {
                alitutdeControl.resetValues(1320 , pressureToAltitude(pressure));
                driveEngines(1000 , *input , RPY{0.f , 0.f , 0.f});
            }

            auto newHeight = map(static_cast<float>(input->switch2) , 1000.f , 2000.f , 0.f, 8.f);
            command.altitude = newHeight + groundAltitude;
        }
    }

    else
        m_engines->failSafe();
}

void FlightControl::controlWJoyistick(const ImuData& rawImuData , const RPY& angles , Receiver& receiver , const ReceiverInput& input , float dt)
{
    auto scaledRollPitchYawInput = receiver.scaleRollPitchYawCommand(input ,m_maxValues);
    auto quadPid = m_pid_quad.getPid(angles , rawImuData , scaledRollPitchYawInput , dt , input.throttle);
    driveEngines(input.throttle , input , quadPid);
}

void FlightControl::driveEngines(float throttle ,const ReceiverInput& input , const RPY& quadPid)
{
    if(throttle > 1035 && input.switch1 > 1500)
    {
        m_engines->driveEngines(throttle , quadPid);
    }
    else
        m_engines->failSafe();
}

bool FlightControl::updateLastThrottleAndControlOwner(ReceiveCommand& command , Control& controller , float throttle , float altitude)
{
    static int count{0};

    if(!m_throttle_has_setted)
    {
        m_last_throttle = throttle;
        controller.setFirstTime(1320 , altitude);
        count = 0;
        m_throttle_has_setted = true;
    }

    if(throttle <= m_last_throttle - 75 || throttle >= m_last_throttle + 75)
        ++count;
    else
        count = 0;

    if(count >= 300)
    {
        Serial.printf("Yes\n");
        command = ReceiveCommand{Command::fly_joyistick , 0 , 0.f , 0.f};
        m_last_throttle = 1275.f;
        m_throttle_has_setted = false;
        controller.setFinishTime();
        count = 0;
        return true;
    }

    return false;
}