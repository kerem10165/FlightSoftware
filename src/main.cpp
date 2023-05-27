#include <Arduino.h>
#include <Wire.h>
#include <Receiver/Receiver.h>
#include <Imu/Imu.h>
#include <FlightControl/FlightControl.h>
#include <AltitudeComputer/AltitudeComputer.h>
#include <Communication/ReceiveCommand.h>
#include <Communication/SendCommand.h>

float dt;
unsigned long current_time, prev_time , time_counter;

Receiver * receiver{nullptr};
Imu* imu{nullptr};
AltitudeComputer* altitudeComputer{nullptr};
FlightControl* flightControl;
TransferData* transferData{};
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

  imu = new Imu{ImuData{0.002604f, 0.001205f , -0.001308f , 2.103643f , 3.661295f ,-0.549706f}};
  altitudeComputer = new AltitudeComputer;
  flightControl = new FlightControl;
  transferData = new TransferData;
  
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


void loop() 
{
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  ImuData rawImuData = imu->getImuData();
  RPY angles = imu->getRollPitchYaw(rawImuData , dt);
  

  auto altitudeValues = altitudeComputer->getAltitudeAndPressure();

  auto baroAltitude = std::get<0>(altitudeValues);
  auto pressure = std::get<1>(altitudeValues);
  auto elapsedTimeLastAltitudeMeasurement = std::get<2>(altitudeValues);

  float groundAltitudeFromPressureCalculation = altitudeComputer->getGroundAltitudeFromPressure();

  flightControl->control(command , rawImuData , angles , elapsedTimeLastAltitudeMeasurement,
                        groundAltitudeFromPressureCalculation, pressure , *receiver , dt);
  

  // Serial.printf("Ground Altitude : %f , Altitude : %f\n" , altitude , altitudeAndPressure.first);

  // Serial.printf("%R : %f, P : %f , Y : %f\n" , angles.Roll , angles.Pitch , angles.Yaw);

  count++;
  end = millis();
  if(end - start > 1000)
  {
    Serial.printf("Count Main : %d\n" , count);
    count2 = count;
    count = 0;
    start = end;
  }

  auto altitudeFromGroundLevel = pressureToAltitude(pressure) - groundAltitudeFromPressureCalculation;

  transferData->transferData(angles , altitudeFromGroundLevel * 100 , command.command , 
  count2);
  loopRate(2026);
}