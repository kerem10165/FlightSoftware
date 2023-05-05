#include <Arduino.h>
#include "Receiver/Receiver.h"

ReceiverInput Receiver::m_inputs{};
int Receiver::m_pin = 0;
unsigned long Receiver::lastReadTime = 0ul;

int ppm_counter = 0;
unsigned long time_ms = 0;

void getPPM();

Receiver::Receiver(int pin)
{
    pinMode(pin, INPUT_PULLUP);
    delay(20);
    attachInterrupt(digitalPinToInterrupt(pin), getPPM, CHANGE);
    m_pin = pin;
}

void getPPM()
{
    unsigned long dt_ppm;
    int trig = digitalRead(Receiver::m_pin);

    if (trig==1) 
    {
        dt_ppm = micros() - time_ms;
        time_ms = micros();

        if (dt_ppm > 5000) 
            ppm_counter = 0;

        else if(ppm_counter > 0 && ppm_counter < 6)
            *(reinterpret_cast<float*>(&Receiver::m_inputs) + ppm_counter - 1) = dt_ppm;

        else if (ppm_counter == 6) 
        {
            *(reinterpret_cast<float*>(&Receiver::m_inputs) + ppm_counter - 1) = dt_ppm;
            Receiver::lastReadTime = micros();
        }

        ppm_counter = ppm_counter + 1;
    }
}


ReceiverInput* Receiver::getCommand()
{
    if(micros() - lastReadTime > 500'000) //timeout
        return nullptr;

    return &m_inputs;
}

RPY Receiver::scaleRollPitchYawCommand(const ReceiverInput& inputs , const RPY& maxValues)
{
    float rollInput{inputs.roll} , pitchInput{inputs.pitch} , yawInput{inputs.yaw};

    if(rollInput > 1490 && rollInput < 1510)
        rollInput = 1500;

    if(pitchInput > 1490 && pitchInput < 1510)
        pitchInput = 1500;

    if(yawInput > 1490 && yawInput < 1510)
        yawInput = 1500;  

    rollInput = (rollInput - 1500.)/500.f;
    pitchInput = (pitchInput - 1500.)/500.f;
    yawInput = (yawInput - 1500.)/500.f;
    
    rollInput = constrain(rollInput , -1.f , 1.f);
    pitchInput = constrain(pitchInput , -1.f , 1.f);
    yawInput = constrain(yawInput , -1.f , 1.f);

    return {rollInput * maxValues.Roll , pitchInput * maxValues.Pitch , yawInput * maxValues.Yaw};
}

void Receiver::printReceiver()
{
    Serial.printf("roll : %f , pitch : %f , yaw : %f , throttle : %f , switch1 : %f , switch2 : %f\n"
    ,m_inputs.roll, m_inputs.pitch , m_inputs.yaw , m_inputs.throttle , m_inputs.switch1 , m_inputs.switch2);
}