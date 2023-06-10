#ifndef MY_GPS_H
#define MY_GPS_H

#include <TinyGPSPlus.h>
#include <utility>

class Gps
{
public:
    using Latitude = float;
    using Longitude = float;
public:
    Gps();
    //to get new data, getCoordinate function has to be called before whatever you want to obtain.
    std::pair<Latitude , Longitude> getCoordinate();
    float getAltitude();
    float getSpeed();
    int getSatelliteCount();
private:
    TinyGPSPlus m_gps;
    float m_lat{} , m_lon{} , m_alt{} , m_speed{};
    int m_satCount{};
};






#endif