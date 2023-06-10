#include <Communication/SendCommand.h>


DebugValues debugValues;


TransferData::TransferData()
{
    m_transfer.begin(details(m_infos), &Serial1);
}

void TransferData::transferData(const RPY& angles , int altitude , Command status , Gps& gps  
                                , int countOfLoop , const QuadSettings& settings)
{
    auto currentSend = millis();
    auto locations = gps.getCoordinate();
    auto satelliteCount = gps.getSatelliteCount();
    if(currentSend - m_lastSend >= 100)
    {
        if(count < 10)
        {
            m_infos.data.status.altitude = altitude;
            m_infos.data.status.status = status;
            m_infos.data.status.roll = angles.Roll;
            m_infos.data.status.pitch = angles.Pitch;
            m_infos.data.status.yaw = angles.Yaw;
            m_infos.data.status.debug = debugValues;
            m_infos.data.status.debug.countOfLoop = countOfLoop;
            
            m_infos.data.status.latitude = locations.first;
            m_infos.data.status.longtitude = locations.second;
            m_infos.data.status.satelliteCount = satelliteCount;
            
            m_infos.sendType = SendType::Info;
            

            count++;
        }
        else
        {
            m_infos.data.settings.altitudeP = settings.m_kp_altitude; 
            m_infos.data.settings.altitudeI = settings.m_ki_altitude; 
            m_infos.data.settings.altitudeD = settings.m_kd_altitude;

            m_infos.data.settings.rollAndPitchP = settings.m_kp_attitude.Roll;
            m_infos.data.settings.rollAndPitchI = settings.m_ki_attitude.Roll;
            m_infos.data.settings.rollAndPitchD = settings.m_kd_attitude.Roll;
            
            m_infos.sendType = SendType::Settings;
            count = 0;
        }

        m_transfer.sendData();
        memset(&m_infos , 0 , sizeof(m_infos));
        m_lastSend = currentSend;
    }
}