#ifndef SETTINGS_H
#define SETTINGS_H

#include <Imu/Imu.h>
#include <FlightControl/FlightControl.h>


class QuadSettings
{
    friend class TransferData;
public:
    QuadSettings(FlightControl* flightControl);
    void getValuesFromEepromAndSetup();
    void getValuesFromEeprom();
    //this code section use for initiliaze eeprom sections and proper sections. 
    //to change settings you have to change params from code then you have to invoke this function
    //but you have to disable getValuesFromEeprom function
    void setValuesWithDefaultParams();
    void setAttitudeControlPid(float p , float i , float d);
    void setAttitudeControlPid(const RPY& p , const RPY& i , const RPY& d);
    void setAltitudePid(float p , float i , float d);
private:
    void setAttitudeControlPid(int address);
    void setAltitudePid(int address);
private:
    FlightControl* m_flightControl;
    RPY m_kp_attitude{0.305f,0.305f,1.4f};
    RPY m_ki_attitude{0.475f,0.475f,0.00105f};
    RPY m_kd_attitude{0.105f,0.105f,0.000075f};
    float m_kp_altitude{0.2f} , m_ki_altitude{0.6f} , m_kd_altitude{0.4f};
    const int m_firstData = 0xABCD;
};




#endif