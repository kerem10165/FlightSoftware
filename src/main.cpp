#include <Arduino.h>
#include <Wire.h>
#include <Receiver/Receiver.h>
#include <Imu/Imu.h>
#include <EngineControl/EngineControl.h>

#ifdef DEBUGOVERWIFI
#include <Debug/DebugDefinitions.h>
#include <EasyTransfer.h>

void sendDebugValues(float throttle);

EasyTransfer ET;
DebugInformation dataToSend;
#endif

#include "IncredibleNewPid/IncredibleNewPid.h"

float dt;
unsigned long current_time, prev_time , time_counter;

RPY maxValues{25.f,25.f,180.f};
RPY kp{0.2,0.2,0.25f};
RPY ki{0.25,0.25,0.05};
RPY kd{0.065,0.065,0.00011};

Receiver * receiver{nullptr};
Imu* imu{nullptr};
EngineControl* engines{nullptr};
IncredibleNewPid newPid{kp,ki,kd};

static inline void loopRate(int freq) 
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

  ET.begin(details(dataToSend), &Serial1);

  imu = new Imu{ImuData{0.04f,-0.02f, -0.02f,-0.56f,-1.03f,-1.54f}};

  receiver = new Receiver{15};
  engines = new EngineControl{12,14,11,13};
  engines->startupEngines();
}


void loop() 
{
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  auto rawImuData = imu->getImuData();
  auto angles = imu->getRollPitchYaw(rawImuData , dt);
  if(auto input = receiver->getCommand())
  {
    auto scaledRollPitchYawInput = receiver->scaleRollPitchYawCommand(maxValues);
    auto pidVal = newPid.getPid(angles , rawImuData , scaledRollPitchYawInput , dt , input->throttle);


    // Serial.printf("Roll : %f Pitch : %f , Yaw : %f\n" , scaledRollPitchYawInput.Roll , scaledRollPitchYawInput.Pitch , scaledRollPitchYawInput . Yaw);

    if(input->throttle > 1060 && input->switch1 > 1500)
    {
      Serial.printf("Roll : %f , Pitch : %f Yaw : %f ", angles.Roll , angles.Pitch , angles.Yaw);
      engines->driveEngine(input->throttle , pidVal);
    }
    else
    {
      engines->failSafe();
    }
  }

  else
  {
    //Serial.println("Error Occured While Reading!!!");
    engines->failSafe();
  }
  loopRate(2000);
}

#ifdef DEBUGOVERWIFI
void sendDebugValues(float throttle)
{
  dataToSend.throttle = throttle;
  dataToSend.engine = engines->m_engineDebug;

  ET.sendData();
}
#endif