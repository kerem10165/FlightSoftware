#include "FlightControl/FlightControl.h"

FlightControl::FlightControl()
{

}

void FlightControl::armEngine()
{
    m_engines = new EngineControl{12,14,11,13};
}

void FlightControl::control(ReceiveCommand& command, const ImuData& rawImuData , const RPY& angles , Receiver& receiver , float dt)
{
    if(auto input = receiver.getCommand())
    {
        if(command.command == Command::fly_joyistick)
        {
            controlWJoyistick(rawImuData , angles , receiver , *input , dt);
            m_throttle_set = true;
        }
        else if(command.command == Command::set_altitude)
        {
            if(m_throttle_set)
            {
                m_last_throttle = input->throttle;
                m_throttle_set = false;
            }
            if(input->throttle < m_last_throttle - 20 || input->throttle > m_last_throttle +20)
            {
                command = ReceiveCommand{Command::fly_joyistick , 0 , 0.f , 0.f};
                m_last_throttle = 1000.f;
                return;
            }

            driveEngines(1050 , *input , {0,0,0});
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