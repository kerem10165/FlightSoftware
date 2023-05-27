#include <AltitudeComputer/AltitudeComputer.h>
#include <limits>


AltitudeComputer::AltitudeComputer()
{
    for(int i = 0 ; i < 5500 ; i++)
        getAltitudeAndPressure();

    int readCount = 1500;

    for(int i = 0 ; i < readCount ; i++)
    {
        auto altitudeValues = getAltitudeAndPressure();

        auto altitude = std::get<0>(altitudeValues);
        auto pressure = std::get<1>(altitudeValues);

        m_groundAlt += altitude;
        m_groundPressure += (pressure /100.f); //prevent to overflow
    }

    m_groundAlt /= readCount;
    (m_groundPressure /= readCount) *= 100.f;
    m_groundAltitudeFromPressure = pressureToAltitude(m_groundPressure);
}

std::tuple<AltitudeComputer::Altitude , AltitudeComputer::Pressure , AltitudeComputer::ElapsedTime> 
    AltitudeComputer::getAltitudeAndPressure()
{
    auto altitudeValues = barometer.readBarometer();
    
    auto altitude = std::get<0>(altitudeValues);
    auto pressure = std::get<1>(altitudeValues);
    auto elapsedTime = std::get<2>(altitudeValues);

    actualPressure = actualPressure * (float)0.985 + pressure * (float)0.015;
    float actual_pressure_diff = actualPressure - pressure;
    if (actual_pressure_diff > 7)actual_pressure_diff = 8;
    if (actual_pressure_diff < -7)actual_pressure_diff = -8;
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actualPressure -= actual_pressure_diff / 6.0;

    return std::make_tuple(altitude , actualPressure , elapsedTime);
}


float pressureToAltitude(float pressure) { return 44330 * (1.0 - pow(pressure / 101325, 0.1903)); }
float altitudeToPressure(float altitude) { return 101325 * pow((1.0 - altitude / 44330), (1.0 / 0.1903)); }
