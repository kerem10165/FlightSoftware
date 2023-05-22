#include <AltitudeComputer/Ultrasonic.h>

UltrasonicSensor::UltrasonicSensor()
{
    while(!URM09.begin())
    {
        Serial.println("I2C device number error ");
        delay(1000);
    }

    URM09.setModeRange(MEASURE_MODE_PASSIVE ,MEASURE_RANG_500);
}

std::pair<UltrasonicSensor::Altitude , UltrasonicSensor::ElapsedTime> UltrasonicSensor::readUltrasonic()
{
    URM09.measurement();
    return URM09.getDistance(); 
}