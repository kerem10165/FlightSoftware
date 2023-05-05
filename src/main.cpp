#include <Arduino.h>
#include <Wire.h>
#include <Receiver/Receiver.h>
#include <Imu/Imu.h>
#include <FlightControl/FlightControl.h>
#include <Communication/ReceiveCommand.h>


float dt;
unsigned long current_time, prev_time , time_counter;

Receiver * receiver{nullptr};
Imu* imu{nullptr};
FlightControl flightControl;
ReceiveCommand command{Command::fly_joyistick , 0 , 0.f , 0.f};


void loopRate(int freq) 
{
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  while (invFreq > (checker - current_time)) 
  {
    checker = micros();
  }
}

void setup() 
{
  Serial.begin(9600);
  Serial1.begin(115200);

  Wire.begin();
  Wire.setClock(1000000);


  imu = new Imu{ImuData{-0.015029f,0.015754f, 0.003937f ,1.967997f,3.844049f,-0.574844f}};
  
  // auto error = imu->getImuError();
  // while(1)
  // {
  //   imu->printImuError(error);
  //   delay(100);
  // }

  receiver = new Receiver{15};
  flightControl.armEngine();

  command.command = Command::set_altitude;
}

uint32_t start , end , count;

void loop() 
{
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;
  
  auto rawImuData = imu->getImuData();
  auto angles = imu->getRollPitchYaw(rawImuData , dt);
  
  flightControl.control(command , rawImuData , angles , *receiver , dt);

  loopRate(10000);
}