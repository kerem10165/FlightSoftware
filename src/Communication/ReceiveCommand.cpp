#include <Communication/ReceiveCommand.h>


ReceiveData::ReceiveData(QuadSettings * quadSettings , ReceiveCommand& receiveCommand)
    :m_quadSettings{quadSettings} , m_receiveCommand{receiveCommand}
{
    m_transfer.begin(details(m_receivedData) , &Serial1);
}

void ReceiveData::receiveDatas()
{
    if(m_transfer.receiveData())
    {
        if(m_receivedData.start[0] == 'o' && m_receivedData.start[1] == 'k'
           && m_receivedData.end[0] == 'o' && m_receivedData.end[1] == 'k')
        {
            if(m_receivedData.type == ReceiveType::ReceiveData)
            {
                m_receiveCommand = m_receivedData.data.receiveCommand;
            }
            
            else if(m_receivedData.type == ReceiveType::ReceiveAltitudePidSetting)
            {
                auto pidValues = m_receivedData.data.altitudePid;
                m_quadSettings->setAltitudePid(pidValues.altitudeP , pidValues.altitudeI , pidValues.altitudeD);
            }

            else if(m_receivedData.type == ReceiveType::ReceiveAttitudePidSetting)
            {
                auto pidValues = m_receivedData.data.attitudePid;
                m_quadSettings->setAttitudeControlPid(pidValues.rollAndPitchP , pidValues.rollAndPitchI , pidValues.rollAndPitchD);
            }
        }
    }
}