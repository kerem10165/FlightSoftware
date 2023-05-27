#include <AltitudeComputer/Barometer.h>
#include <limits>
BarometerSensor::BarometerSensor()
{
    bmp388.begin(0x76);
    bmp388.setPresOversampling(OVERSAMPLING_X8);
    bmp388.setTempOversampling(OVERSAMPLING_SKIP);
    bmp388.setIIRFilter(IIR_FILTER_8);
}

std::tuple<BarometerSensor::Altitude , BarometerSensor::Pressure , BarometerSensor::ElapsedTime> BarometerSensor::readBarometer()
{
    bmp388.startForcedConversion();  
    if (bmp388.getMeasurements(m_temperature, m_pressure, m_altitude))
    {
        current_read = millis();
        auto elapsed_time = current_read - last_read;
        last_read = current_read;
        return std::make_tuple(m_altitude , m_pressure*100.f , elapsed_time);
    }

    return std::make_tuple(m_altitude , m_pressure*100.f , std::numeric_limits<uint32_t>::max());
}