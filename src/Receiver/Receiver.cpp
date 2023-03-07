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

RPY Receiver::scaleRollPitchYawCommand(const RPY& maxValues)
{
    return 
        {   map(m_inputs.roll , 990 , 2010 , -maxValues.Roll , maxValues.Roll),
            -map(m_inputs.pitch , 990 , 2010 , -maxValues.Pitch , maxValues.Pitch),
            map(m_inputs.yaw , 990 , 2010 , -maxValues.Yaw , maxValues.Yaw)  };
}

void Receiver::printReceiver()
{
    Serial.printf("roll : %f , pitch : %f , yaw : %f , throttle : %f , switch1 : %f , switch2 : %f\n"
    ,m_inputs.roll, m_inputs.pitch , m_inputs.yaw , m_inputs.throttle , m_inputs.switch1 , m_inputs.switch2);
}