#include <Arduino.h>
#include <Receiver/Receiver.h>
#include <Imu/Imu.h>
#include <Pid/Pid.h>
#include <EngineControl/EngineControl.h>

float dt;
unsigned long current_time, prev_time;

RPY maxValues{40.f,40.f,180.f};
RPY kp{0.25,0.25,0.5};
RPY ki{0.8,0.8,0.01};
RPY kd{0.025,0.025,0.};

Receiver * receiver{nullptr};
Imu* imu{nullptr};
EngineControl* engines{nullptr};
Pid pid{kp,ki,kd};

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

  imu = new Imu{2000};

  receiver = new Receiver{15};
  engines = new EngineControl{12,14,11,13};
  engines->startupEngines();
}

void loop() 
{
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  if(auto input = receiver->getCommand())
  {
    if(input->throttle > 1060 && input->throttle < 1080 && input->switch1 > 1500)
      pid.resetIntegralValues();

    if(input->throttle > 1060 && input->switch1 > 1500)
    {
      auto gyroData = imu->getRollPitchGyroZ();
      auto scaledRollPitchYawInput = receiver->scaleRollPitchYawCommand(maxValues);
      auto pidVal = pid.getPidValues(gyroData , scaledRollPitchYawInput , 1.f / 250);

      Serial.printf("%Roll: %f , Pitch : %f , Yaw : %f , Throttle : %f\n" , scaledRollPitchYawInput.Roll , 
      scaledRollPitchYawInput.Pitch , scaledRollPitchYawInput.Yaw , input->throttle);

      engines->driveEngine(input->throttle , pidVal);
      Serial.printf("\n\n");
    }
    else
    {
      engines->failSafe();
      pid.resetPidValues();
    }
  }

  else
  {
    Serial.println("Error Occured While Reading!!!");
    engines->failSafe();
    pid.resetPidValues();
  }
  
  loopRate(250);
}

