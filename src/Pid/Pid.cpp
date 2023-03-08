#include <Arduino.h>
#include <Pid/Pid.h>



Pid::Pid(const RPY& kp , const RPY& ki , const RPY& kd , float integralLimit , float maxPidOutputLimit)
    :m_kp{kp} , m_ki{ki} , m_kd{kd} , m_integralLimit{integralLimit} , m_maxPidOutputLimit{maxPidOutputLimit}
{
    
}

RPY Pid::getPidValues(const RPY &angels, const RPY &desiredAngles, float dt)
{
    Serial.print("Roll ");
    float rollPid = getPidValue(angels.Roll , desiredAngles.Roll , m_kp.Roll , m_ki.Roll , 
    m_kd.Roll , m_lastError.Roll, m_lastIntegral.Roll , dt , DebugChoice::Roll);

    Serial.print("Pitch ");
    float pitchPid = getPidValue(angels.Pitch , desiredAngles.Pitch , m_kp.Pitch , m_ki.Pitch , 
    m_kd.Pitch , m_lastError.Pitch, m_lastIntegral.Pitch , dt , DebugChoice::Pitch);
    
    Serial.print("Yaw ");
    float yawPid = getPidValue(angels.Yaw , desiredAngles.Yaw , m_kp.Yaw , m_ki.Yaw , 
    m_kd.Yaw , m_lastError.Yaw, m_lastIntegral.Yaw , dt ,  DebugChoice::Yaw);

    Serial.println();

    return RPY{rollPid , pitchPid , yawPid};
}

void Pid::resetPidValues()
{
    m_lastError = RPY{};
    m_lastIntegral = RPY{};
}

void Pid::resetIntegralValues()
{
    m_lastIntegral = RPY{};
}

float Pid::getPidValue(float angle , float desiredAngle , float P , float I , float D,
     float& lastError , float& lastIntegral , float dt , DebugChoice choice)
{
    float errorRate = desiredAngle - angle;

    float Pterm=P*errorRate;
    float Iterm= lastIntegral+ I*(errorRate)*dt;
    
    Iterm = constrain(Iterm , -m_integralLimit , m_integralLimit);

    float Dterm=D*((errorRate-lastError)/dt);

    float pidOutput= Pterm+Iterm+Dterm;
    pidOutput = constrain(pidOutput , -m_maxPidOutputLimit , m_maxPidOutputLimit);

    Serial.printf(" Desired Angle : %f , Angle : %f , Integral : %f "
    ", IntegralPrev : %f , %f Output : %f\n" , 
    desiredAngle , angle , Iterm , lastIntegral , pidOutput);

    lastError = errorRate;
    lastIntegral = Iterm;

    return pidOutput;
}