#include <Arduino.h>
#include <Servo.h>
#include <Imu/Imu.h>
#include <Receiver/Receiver.h>
#include <Pid/Pid.h>
#include <EngineControl/EngineControl.h>


float dt;
unsigned long current_time, prev_time;

RPY maxValues{30.f,30.f,180.};
RPY kp{0.3,0.3,0.3};
RPY ki{0.3,0.3,0.05};
RPY kd{0.05,0.05,0.00015};

Imu* imu;
Receiver * receiver;
Pid pid{kp,ki,kd , 25.f};
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
    auto scaledRollPitchYawInput = receiver->scaleRollPitchYawCommand(maxValues);
    auto pidVal = pid.getPidValues(rollPitchYaw , accelAndGyro , scaledRollPitchYawInput , dt);

    engines->driveEngine(input->throttle , pidVal);
  }

  else
  {
    Serial.println("Error!!!!");
    engines->failSafe();
  }
  
  

  loopRate(2000);
}

