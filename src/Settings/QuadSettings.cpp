#include <Settings/QuadSettings.h>
#include <EEPROM.h>


QuadSettings::QuadSettings(FlightControl* flightControl)
    :m_flightControl{flightControl}
{
    
}

void  QuadSettings::getValuesFromEepromAndSetup()
{
    getValuesFromEeprom();

    m_flightControl->setAttitudeControlPid(m_kp_attitude , m_ki_attitude , m_kd_attitude);
    m_flightControl->setAltitudeControlPid(m_kp_altitude , m_ki_altitude , m_kd_altitude);
}

void QuadSettings::getValuesFromEeprom()
{
    int firstData;
    int address = 0;
    if(EEPROM.get(address, firstData) == m_firstData)
    {
        address += sizeof(m_firstData);
        EEPROM.get(address , m_kp_attitude);
        address += sizeof(m_kp_attitude);
        EEPROM.get(address , m_ki_attitude);
        address += sizeof(m_ki_attitude);
        EEPROM.get(address , m_kd_attitude);
        address += sizeof(m_kd_attitude);
        
        EEPROM.get(address , m_kp_altitude);
        address += sizeof(m_kp_altitude);
        EEPROM.get(address , m_ki_altitude);
        address += sizeof(m_ki_altitude);
        EEPROM.get(address , m_kd_altitude);
    }
}

void QuadSettings::setValuesWithDefaultParams()
{
    int address = 0;
    EEPROM.put(address , m_firstData);
    address += sizeof(m_firstData);
    setAttitudeControlPid(address);
    address += 3 * sizeof(m_kd_attitude);
    setAltitudePid(address);

    m_flightControl->setAttitudeControlPid(m_kp_attitude , m_ki_attitude , m_kd_attitude);
    m_flightControl->setAltitudeControlPid(m_kp_altitude , m_ki_altitude , m_kd_altitude);
}

void QuadSettings::setAttitudeControlPid(float p , float i , float d)
{
    m_kp_attitude.Pitch = p;
    m_kp_attitude.Roll = p;
    
    m_ki_attitude.Pitch = i;
    m_ki_attitude.Roll = i;
    
    m_kd_attitude.Pitch = d;
    m_kd_attitude.Roll = d;

    setAttitudeControlPid(m_kp_attitude ,m_ki_attitude , m_kd_attitude);
}

void QuadSettings::setAttitudeControlPid(const RPY& p , const RPY& i , const RPY& d)
{
    m_kp_attitude = p;
    m_ki_attitude = i;
    m_kd_attitude = d;


    int address = sizeof(m_firstData);
    setAttitudeControlPid(address);
    m_flightControl->setAttitudeControlPid(m_kp_attitude , m_ki_attitude , m_kd_attitude);
}

void QuadSettings::setAltitudePid(float p , float i , float d)
{
    m_kp_altitude = p;
    m_ki_altitude = i;
    m_kd_altitude = d;

    int address = sizeof(m_firstData) + 3 * sizeof(m_kd_attitude);
    setAltitudePid(address);
    m_flightControl->setAltitudeControlPid(m_kp_altitude , m_ki_altitude , m_kd_altitude);
}

void QuadSettings::setAttitudeControlPid(int address)
{
    EEPROM.put(address , m_kp_attitude);
    address += sizeof(m_kp_attitude);
    EEPROM.put(address , m_ki_attitude);
    address += sizeof(m_ki_attitude);
    EEPROM.put(address , m_kd_attitude);
    address += sizeof(m_kd_attitude);
}

void QuadSettings::setAltitudePid(int address)
{
    EEPROM.put(address , m_kp_altitude);
    address += sizeof(m_kp_altitude);
    EEPROM.put(address , m_ki_altitude);
    address += sizeof(m_ki_altitude);
    EEPROM.put(address , m_kd_altitude);
}