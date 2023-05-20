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
FlightControl* flightControl;
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
  Wire.setClock(400'000);


  imu = new Imu{ImuData{-0.005709f,-0.012842f, -0.001831f ,2.307630f,3.684293f,-0.554160f}};
  flightControl = new FlightControl;
  // auto error = imu->getImuError();
  // while(1)
  // {
  //   imu->printImuError(error);
  //   delay(100);
  // }

  receiver = new Receiver{15};
  flightControl->armEngine();
  


  command.command = Command::set_altitude;
  command.altitude = 1.75;
}

uint32_t start , end , count;


//Sonardan gelen verileri filtrele
 
void loop() 
{
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  ImuData rawImuData = imu->getImuData();
  RPY angles = imu->getRollPitchYaw(rawImuData , dt);
  flightControl->control(command , rawImuData , angles , *receiver , dt);

  

  // Serial.printf("%R : %f, P : %f , Y : %f\n" , angles.Roll , angles.Pitch , angles.Yaw);

  // count++;
  // end = millis();
  // if(end - start > 1000)
  // {
  //   Serial.printf("Count Main : %d\n" , count);
  //   count = 0;
  //   start = end;
  // }

  loopRate(2000);
}