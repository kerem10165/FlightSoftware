#include <AltitudeComputer/AltitudeComputer.h>
#include <limits>

AltitudeComputer::AltitudeComputer()
{

}

std::pair<AltitudeComputer::Altitude , AltitudeComputer::Velocity> AltitudeComputer::getAltitudeAndVerticalVelocity()
{
    auto altBaram = barometer.readBarometer();
    auto altUltrasonic = ultrasonic.readUltrasonic();

    uint32_t currentMeasurementTime = std::numeric_limits<uint32_t>::max();

    if(altUltrasonic.first >= 0 && altUltrasonic.first <= 275)
    {
        m_groundAltBarom = altBaram.first - altUltrasonic.first;
        if(altUltrasonic.second != std::numeric_limits<uint32_t>::max())
        {
           currentMeasurementTime = millis();
        } 
    }

    else if(altBaram.second != std::numeric_limits<uint32_t>::max())
    {
        currentMeasurementTime = millis();
    }

    if(currentMeasurementTime != std::numeric_limits<uint32_t>::max())
    {
        m_velocity = getVelocity(currentMeasurementTime , altBaram.first - m_groundAltBarom ,
         m_lastMeasurementTime , m_lastAltitude);

         m_lastMeasurementTime = currentMeasurementTime;
    }

    // if(altBaram.second != std::numeric_limits<uint32_t>::max())
    //     Serial.printf("Barom : %f , Velocity : %f , elapsed time: %d\n" , altBaram.first /100.f  , m_velocity , altBaram.second);

    m_lastAltitude = altBaram.first - m_groundAltBarom;


    return std::make_pair(altBaram.first - m_groundAltBarom , m_velocity);
}

float AltitudeComputer::getVelocity(uint32_t currentMeasurementTime , int currentAltitude , uint32_t lastMeasurementTime , int lastAltitude)
{
    return static_cast<float>(currentAltitude - lastAltitude) / ((currentMeasurementTime - lastMeasurementTime) /1000.f);
}