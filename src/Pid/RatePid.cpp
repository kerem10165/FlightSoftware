#include <Arduino.h>
#include <Pid/RatePid.h>



RatePid::RatePid(const RPY& kp , const RPY& ki , const RPY& kd , float integralLimit , float maxPidOutputLimit)
    :m_kp{kp} , m_ki{ki} , m_kd{kd} , m_integralLimit{integralLimit} , m_maxPidOutputLimit{maxPidOutputLimit}
{
    
}

RPY RatePid::getPidValues(const RPY &angels, const RPY &desiredAngles, float dt)
{
    Serial.print("Roll ");
    float rollPid = getPidValue(angels.Roll , desiredAngles.Roll , m_kp.Roll , m_ki.Roll , 
    m_kd.Roll , m_lastError.Roll, m_lastIntegral.Roll , dt);

    Serial.print("Pitch ");
    float pitchPid = getPidValue(angels.Pitch , desiredAngles.Pitch , m_kp.Pitch , m_ki.Pitch , 
    m_kd.Pitch , m_lastError.Pitch, m_lastIntegral.Pitch , dt);
    
    Serial.print("Yaw ");
    float yawPid = getPidValue(angels.Yaw , desiredAngles.Yaw , m_kp.Yaw , m_ki.Yaw , 
    m_kd.Yaw , m_lastError.Yaw, m_lastIntegral.Yaw , dt);

    Serial.println();

    return RPY{rollPid , pitchPid , yawPid};
}

void RatePid::resetPidValues()
{
    m_lastError = RPY{};
    m_lastIntegral = RPY{};
}

void RatePid::resetIntegralValues()
{
    m_lastIntegral = RPY{};
}

float RatePid::getPidValue(float angle , float desiredAngle , float P , float I , float D,
     float& lastError , float& lastIntegral , float dt)
{
    float errorRate = desiredAngle - angle;

    float Pterm=P*errorRate;
    float Iterm= lastIntegral+ I*(errorRate+lastError)*dt/2;

    if (Iterm > m_integralLimit) Iterm=m_integralLimit;
    else if (Iterm <-m_integralLimit) Iterm=-m_integralLimit;
    
    float Dterm=D*(errorRate-lastError)/dt;
    
    float pidOutput= Pterm+Iterm+Dterm;

    if (pidOutput>m_maxPidOutputLimit) pidOutput=m_maxPidOutputLimit;
    else if (pidOutput <-m_maxPidOutputLimit) pidOutput=-m_maxPidOutputLimit;

    Serial.printf(" Desired Angle : %f , Angle : %f , Integral : %f "
    ", IntegralPrev : %f , dt : %f kp : %f , ki : %f , kd : %f Output : %f\n" , 
    desiredAngle , angle , Iterm , lastIntegral , dt , P , I , D , pidOutput);


    lastError = errorRate;
    lastIntegral = Iterm;

    return pidOutput;
}