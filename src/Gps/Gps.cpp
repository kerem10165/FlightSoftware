#include <Gps/Gps.h>


Gps::Gps()
{
    Serial2.begin(9600);
}

std::pair<Gps::Latitude , Gps::Longitude> Gps::getCoordinate()
{
    if(Serial2.available() > 0)
    {
        if (m_gps.encode(Serial2.read()))
        {
            if (m_gps.location.isValid())
            {
                m_lat = m_gps.location.lat();
                m_lon = m_gps.location.lng();
            }
        }
    }

    return std::make_pair(m_lat , m_lon);
}

float Gps::getAltitude()
{
    if(m_gps.altitude.isValid())
        m_alt = m_gps.altitude.meters();
    
    return m_alt;
}

float Gps::getSpeed()
{
    if(m_gps.speed.isValid())
        m_speed = m_gps.speed.kmph();

    return m_speed;
}

int Gps::getSatelliteCount()
{
    if(m_gps.satellites.isValid())
        m_satCount = m_gps.satellites.value();

    return m_satCount;
}