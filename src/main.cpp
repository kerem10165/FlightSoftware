#include <Arduino.h>
#include <Servo.h>
#include <Imu/Imu.h>
#include <Receiver/Receiver.h>
#include <Pid/Pid.h>
#include <EngineControl/EngineControl.h>


float dt;
unsigned long current_time, prev_time;

RPY maxValues{30.f,30.f,180.};
RPY kp{0.5,0.5,0.4};
RPY ki{0.2,0.2,0.5};
RPY kd{0.1,0.1,0.0015};

Imu* imu;
Receiver * receiver;
Pid pid{kp,ki,kd , 50.f};
EngineControl* engines;


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
  imu = new Imu;

  imu->printImuError();

  engines = new EngineControl{12,14,11,13};
  engines->startupEngines();
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
    if(input->throttle >= 1012 && input->switch1 > 1500)
    {
      auto scaledRollPitchYawInput = receiver->scaleRollPitchYawCommand(maxValues);
      auto pidVal = pid.getPidValues(rollPitchYaw , accelAndGyro , scaledRollPitchYawInput , input->throttle , dt);

      //Serial.printf("Kumanda Roll : %f , Pitch : %f , Yaw : %f , Throttle : %f\n" , input->roll ,input->pitch , input->yaw , input->throttle);
      Serial.printf("Angels Roll : %f , Pitch : %f , Yaw : %f\n" , rollPitchYaw.roll , rollPitchYaw.pitch , rollPitchYaw.yaw);

      engines->driveEngine(input->throttle , pidVal);
    }
    else
    {
      engines->startupEngines();
      pid.resetLastPidValues();
    }   
  }

  else
  {
    Serial.println("Error!!!!");
    engines->failSafe();
    pid.resetLastPidValues();
  }
  
  

  loopRate(2000);
}

