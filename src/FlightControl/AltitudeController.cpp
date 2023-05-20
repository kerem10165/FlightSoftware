#include <FlightControl/AltitudeController.h>
#include <limits>

AltitudeController::AltitudeController() : URM09{}
{
    while(!URM09.begin()){
        Serial.println("I2C device number error ");
        delay(1000);
    }

  URM09.setModeRange(MEASURE_MODE_PASSIVE ,MEASURE_RANG_500);

  setStartAltitude();
}

float AltitudeController::control(ReceiveCommand& command)
{
    auto mes = readUltrasonic();
    
    m_filteredAlt = m_altitudeFilterCof*mes.first + (1.f - m_altitudeFilterCof) * m_filteredAlt; 

    if(m_filteredAlt < 0)
        return 1200.f;
    if(mes.second != std::numeric_limits<uint32_t>::max())
    {
        auto dif = (m_filteredAlt - m_lastAltitude);
        auto vel = dif / (mes.second / 1000.f);
        m_lastAltitude = m_filteredAlt;
        vel = constrain(vel , -1000 , 1000); // prevent to wrong measurement
        float thro_pid = m_pid.pid(command.altitude * 100 , m_filteredAlt , vel);
        m_lastPid = thro_pid;
        auto newThro = m_throttle + thro_pid;
        newThro = constrain(newThro , 1225 , 1750);
        return newThro;
    }

    auto newThro = m_throttle + m_lastPid;
    newThro = constrain(newThro , 1225 , 1750);

    return newThro;
}

void AltitudeController::setFirstTime(float throttle)
{
    m_throttle = throttle;
    m_lastAltitude = 0.f;
    m_lastPid = 0.f;
    m_pid = AltitudePid{};
}

void AltitudeController::setFinishTime()
{
    m_throttle = 1000.f;
    m_lastAltitude = 0.f;
    m_lastPid = 0.f;
    m_pid = AltitudePid{};
}

std::pair<int16_t , uint32_t> AltitudeController::readUltrasonic()
{
    URM09.measurement();
    return URM09.getDistance(); 
}

void AltitudeController::setStartAltitude()
{
    while(1)
    {
        auto mes = readUltrasonic();
        if(mes.second != std::numeric_limits<uint32_t>::max() && mes.first >= 0)
        {
            m_filteredAlt = mes.first;
            return; 
        }
    }
}


float AltitudePid::pid(float desiredAltitude , float altitude , float velocity)
{
    float error = desiredAltitude - altitude;

    float proportional = error * m_kp;

    proportional = constrain(proportional , -50 , 200);
    
    float integral = m_integralPrev + error * 0.04;
    integral = constrain(integral , -50 , 175);
    m_integralPrev = integral;

    float derivative = velocity * m_kd;
    derivative = constrain(derivative , -50 , 200);
    m_errorPrev = error;

    return proportional + integral*m_ki - derivative;
}