#include <Arduino.h>
#include <Receiver/Receiver.h>



float dt;
unsigned long current_time, prev_time;


Receiver * receiver;


static inline void loopRate(int freq) 
{
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void setup() {
  Serial.begin(9600);

  receiver = new Receiver{15};
}

void loop() 
{
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  if(auto input = receiver->getCommand())
  {
    receiver->printReceiver();
  }

  else
  {
    Serial.println("Error Occured While Reading!!!");
  }
  
  loopRate(2000);
}

