#include <Communication/SendCommand.h>

DebugValues debugValues;

TransferData::TransferData()
{
    m_transfer.begin(details(m_infos), &Serial1);
}

void TransferData::transferData(const RPY& angles , int altitude , Command status , int countOfLoop)
{
    auto currentSend = millis();
    if(currentSend - m_lastSend >= 75)
    {
        m_infos.altitude = altitude;
        m_infos.status = status;
        m_infos.roll = angles.Roll;
        m_infos.pitch = angles.Pitch;
        m_infos.yaw = angles.Yaw;
        m_infos.satelliteCount = countOfLoop;
        m_infos.debug = debugValues;
        m_infos.debug.countOfLoop = countOfLoop;

        m_transfer.sendData();
        m_lastSend = currentSend;
    }
}