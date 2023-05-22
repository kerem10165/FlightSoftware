#include <AltitudeComputer/Barometer.h>
#include <limits>
BarometerSensor::BarometerSensor()
{
    bmp388.begin(0x76);
    // bmp388.setTimeStandby(TIME_STANDBY_20MS);
    // bmp388.startNormalConversion();
    bmp388.setPresOversampling(OVERSAMPLING_X8);
    bmp388.setTempOversampling(OVERSAMPLING_X4);
    bmp388.setIIRFilter(IIR_FILTER_16);
}

std::pair<BarometerSensor::Altitude , BarometerSensor::ElapsedTime> BarometerSensor::readBarometer()
{
    bmp388.startForcedConversion();  
    if (bmp388.getMeasurements(m_temperature, m_pressure, m_altitude))
    {
        current_read = millis();
        auto elapsed_time = current_read - last_read;
        last_read = current_read;
        return std::make_pair(static_cast<int>(m_altitude*100) , elapsed_time);
    }

    return std::make_pair(static_cast<int>(m_altitude*100) , std::numeric_limits<uint32_t>::max());
}