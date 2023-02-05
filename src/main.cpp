#include <Arduino.h>
#include <Servo.h>
#include <Imu/Imu.h>
#include <Receiver/Receiver.h>
#include <Pid/Pid.h>


float dt;
unsigned long current_time, prev_time;

RPY maxValues{30.f,30.f,180.};

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
  imu = new Imu;

  imu->printImuError();
}


void loop() 
{
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  const auto& accelAndGyro = imu->getImuData();

  const auto& rollPitchYaw = imu->getRollPitchYaw(accelAndGyro , dt);

  if(auto input = receiver->getCommand())
  {
    auto scaledRollPitchYawInput = receiver->scaleRollPitchYawCommand(maxValues);
    Serial.printf("roll : %f , pitch : %f , yaw : %f\n" , scaledRollPitchYawInput.roll , scaledRollPitchYawInput.pitch , scaledRollPitchYawInput.yaw);
  }

  else
  {
    Serial.println("Error!!!!");
  }
  

  loopRate(2000);
}

