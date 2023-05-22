#include <FlightControl/AltitudeController.h>
#include <limits>

AltitudeController::AltitudeController()
{

}

float AltitudeController::control(ReceiveCommand& command ,
                const std::pair<AltitudeComputer::Altitude , AltitudeComputer::Velocity>& altitudeAndVelocity)
{
    auto velocity = constrain(altitudeAndVelocity.second , -1000 , 1000);
    float thro_pid = m_pid.pid(command.altitude * 100 , altitudeAndVelocity.first , velocity);
    auto newThro = m_throttle + thro_pid;
    newThro = constrain(newThro , 1225 , 1750);
    return newThro;
}

void AltitudeController::setFirstTime(float throttle)
{
    m_throttle = throttle;
    m_pid = AltitudePid{};
}

void AltitudeController::setFinishTime()
{
    m_throttle = 1000.f;
    m_pid = AltitudePid{};
}



float AltitudePid::pid(float desiredAltitude , float altitude , float velocity)
{
    float error = desiredAltitude - altitude;

    float proportional = error * m_kp;

    proportional = constrain(proportional , -50 , 200);
    
    float integral = m_integralPrev + error * 0.00045f;
    integral = constrain(integral , -50 , 175);
    m_integralPrev = integral;

    float derivative = velocity * m_kd;
    derivative = constrain(derivative , -50 , 175);
    m_errorPrev = error;

    return proportional + integral*m_ki - derivative;
}