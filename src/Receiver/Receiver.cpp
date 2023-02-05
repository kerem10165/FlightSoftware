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