#include <Arduino.h>
#include <Wire.h>
#include <Receiver/Receiver.h>
#include <Imu/Imu.h>
#include <FlightControl/FlightControl.h>
#include <AltitudeComputer/AltitudeComputer.h>
#include <Communication/ReceiveCommand.h>

float dt;
unsigned long current_time, prev_time , time_counter;

Receiver * receiver{nullptr};
Imu* imu{nullptr};
AltitudeComputer* altitudeComputer{nullptr};
FlightControl* flightControl;
ReceiveCommand command{Command::fly_joyistick , 0 , 0.f , 0.f};

#include <DebugDefinitions.h>
DebugInformation deb;

#include <EasyTransfer.h>

EasyTransfer ET , ET_in; 



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
  ET.begin(details(deb), &Serial1);


  Wire.begin();
  Wire.setClock(400'000);


  imu = new Imu{ImuData{0.021339f, -0.006140f , -0.002601f , 2.179961 , 3.792236f ,-0.551685f}};
  flightControl = new FlightControl;
  altitudeComputer = new AltitudeComputer;

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

uint32_t start , end , count , count2;

uint32_t sendCountStart, sendCountEnd;

//Sonardan gelen verileri filtrele
 
void loop() 
{
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  ImuData rawImuData = imu->getImuData();
  RPY angles = imu->getRollPitchYaw(rawImuData , dt);
  auto altitudeAndVerticalVelocity = altitudeComputer->getAltitudeAndVerticalVelocity();
  flightControl->control(command , rawImuData , angles , altitudeAndVerticalVelocity , *receiver , dt);
  
  Serial.printf("%R : %f, P : %f , Y : %f\n" , angles.Roll , angles.Pitch , angles.Yaw);

  sendCountEnd = millis();
  if(sendCountEnd - sendCountStart >= 100)
  {
    ET.sendData();
    sendCountStart = sendCountEnd;
  }


  count++;
  end = millis();
  if(end - start > 1000)
  {
    deb.throttle = (float)count;
    Serial.printf("Count Main : %d\n" , count);
    count = 0;
    start = end;
  }

  loopRate(2060);
}