#include <FlightControl/AltitudeController.h>
#include <AltitudeComputer/AltitudeComputer.h>
#include <Communication/SendCommand.h>
#include <Communication/ReceiveCommand.h>
#include <limits>

const int AltitudeController::verticalReadCount = 25;

extern DebugValues debugValues;

AltitudeController::AltitudeController()
    :m_verticalSpeeds{std::deque<float>(verticalReadCount , 0.f)}
{

}

float AltitudeController::control(float desiredAltitude ,float pressure , uint32_t elapsedTimeLastAltitudeMeasurement)
{
    auto altitude = pressureToAltitude(pressure);

    if(elapsedTimeLastAltitudeMeasurement != std::numeric_limits<uint32_t>::max())
    {
        auto verticalSpeed = (altitude - m_lastAltitude) / (elapsedTimeLastAltitudeMeasurement / 1000.f);
        m_accumuleteVerticalSpeed += verticalSpeed;
        m_verticalSpeeds.push(verticalSpeed);
        m_accumuleteVerticalSpeed -= m_verticalSpeeds.front();
        m_verticalSpeeds.pop();
        m_lastAltitude = altitude;
    }
    
    auto verticalSpeed = (m_accumuleteVerticalSpeed / verticalReadCount) * 100; 
    verticalSpeed = constrain(verticalSpeed , -1000 , 1000);
    
    float thro_pid = m_pid.pid(desiredAltitude * 100 , altitude *100.f , verticalSpeed);
    auto newThro = m_throttle + thro_pid;
    newThro = constrain(newThro , 1250 , 1750);

    return newThro;
}

void AltitudeController::resetValues(float throttle , float altitude)
{
    m_countOfReset++;
    if(m_countOfReset >= 300)
    {
        setFirstTime(throttle , altitude);
    }
}

void AltitudeController::setFirstTime(float throttle , float altitude)
{
    m_throttle = throttle;
    m_pid = AltitudePid{};
    m_verticalSpeeds = std::queue<float>{std::deque<float>(verticalReadCount , 0.f)};
    m_lastAltitude = altitude;
    m_accumuleteVerticalSpeed = 0.f;
    m_countOfReset = 0;
}

void AltitudeController::setFinishTime()
{
    m_throttle = 1000.f;
    m_pid = AltitudePid{};
    m_verticalSpeeds = std::queue<float>{std::deque<float>(verticalReadCount , 0.f)};
    m_lastAltitude = 0.f;
    m_accumuleteVerticalSpeed = 0.f;
    m_countOfReset = 0;
}

AltitudePid::AltitudePid(AltitudePid&& other)
{
    m_integralPrev = 0.f;
    m_errorPrev = 0.f;
}

AltitudePid& AltitudePid::operator=(AltitudePid&& other)
{
    m_integralPrev = 0.f;
    m_errorPrev = 0.f;
    return *this;
}


void AltitudePid::setPidParams(float p , float i , float d)
{
    m_kp = p;
    m_ki = i;
    m_kd = d;
}

float AltitudePid::pid(float desiredAltitude , float altitude , float velocity)
{
    float error = desiredAltitude - altitude;
    
    float pid_error_gain_altitude = 0;
    if (error > 150 || error < -150) 
    {
        pid_error_gain_altitude = (abs(error) - 150) / 350.0;
        if (pid_error_gain_altitude > 0.4f)
            pid_error_gain_altitude = 0.4f;
    }

    float proportional = error * (m_kp + pid_error_gain_altitude);

    proportional = constrain(proportional , -50 , 200);
    
    float integral = m_integralPrev + error * 0.0001f;
    integral = constrain(integral , -80 , 300);
    m_integralPrev = integral;

    float derivative = velocity * m_kd;
    derivative = constrain(derivative , -150 , 125);
    m_errorPrev = error;

    debugValues.altitude = altitude;
    debugValues.desired_altitude = desiredAltitude;
    debugValues.p_gain_alt = pid_error_gain_altitude;
    debugValues.P_alt = proportional;  
    debugValues.I_alt = integral*m_ki;    
    debugValues.D_alt = derivative;    

    return proportional + integral*m_ki - derivative;
}

