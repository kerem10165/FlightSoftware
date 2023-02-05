#include <Arduino.h>
#include <Servo.h>
#include <Imu/Imu.h>
#include <Receiver/Receiver.h>


float dt;
unsigned long current_time, prev_time;

Imu* imu;
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

  receiver = new Receiver{23};

  /*imu = new Imu;

  imu->printImuError();*/
}


void loop() 
{
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  /*const auto& accelAndGyro = imu->getImuData();

  const auto& rollPitchYaw = imu->getRollPitchYaw(accelAndGyro , dt);*/

  auto input = receiver->getCommand();

  if(input)
  {
    Serial.printf("roll : %f , pitch : %f , yaw : %f , throttle : %f , switch1 : %f , switch2 : %f\n"
    ,input->roll, input->pitch , input->yaw , input->throttle , input->switch1 , input->switch2);
  }

  else
  {
    Serial.println("Error!!!!");
  }
  

  loopRate(2000);
}

