#include "DFRobot_URM09.h"
#include <limits>
DFRobot_URM09::DFRobot_URM09()
{
}

DFRobot_URM09::~DFRobot_URM09()
{
}

bool DFRobot_URM09::begin(uint8_t address)
{
  this->_addr = address;
  // Wire.begin();
  Wire.beginTransmission(_addr);
  if(Wire.endTransmission() == 0)
    return true;
  return false;
}

void DFRobot_URM09::setModeRange(uint8_t range ,uint8_t mode)
{  
  txbuf[0] = (uint8_t)(range | mode);
  i2cWriteTemDistance(eCFG_INDEX, &txbuf[0], 1);
}

void DFRobot_URM09::measurement()
{ 
    if(process == 0)
    {
        end = millis();
        if(end - start > 1)
        {
            txbuf[0] = CMD_DISTANCE_MEASURE;
            i2cWriteTemDistance(eCMD_INDEX, &txbuf[0], 1);
            start = end;
            process = 1;
        }
    }
 
}

float  DFRobot_URM09::getTemperature()
{
  uint8_t i = 0;
  uint8_t rxbuf[10] = {0};
  Wire.beginTransmission(_addr);
  Wire.write(eTEMP_H_INDEX);
  Wire.endTransmission();
  Wire.requestFrom(_addr, (uint8_t)2);
  while (Wire.available()){
    rxbuf[i++] = Wire.read();
  }
  return (((int16_t)rxbuf[0] << 8) + rxbuf[1]) / 10;
}

std::pair<int16_t , uint32_t> DFRobot_URM09::getDistance()
{
  uint8_t i = 0;
  
    if(process == 1)
    {
        end = millis();
        if(end - start > 5)
        {
            Wire.beginTransmission(_addr);
            Wire.write(eDIST_H_INDEX);
            Wire.endTransmission();
            start = end;
            process = 2;
        }
    }

    if(process == 2)
    {
        end = millis();
        if(end - start > 20)
        {
            Wire.requestFrom(_addr, (uint8_t)2);
            while (Wire.available()){
                rxbuf[i++] = Wire.read();
            }
            start = end;
            auto dif_mes = end - last_mesurment;
            last_mesurment = end;
            process = 0;
            int16_t res = ((int16_t)rxbuf[0] << 8) + rxbuf[1];

            return std::make_pair(res , dif_mes);
        }
    }

    int16_t res = ((int16_t)rxbuf[0] << 8) + rxbuf[1];

    return std::make_pair(res , std::numeric_limits<uint32_t>::max());
}

void DFRobot_URM09::i2cWriteTemDistance(uint8_t Reg , uint8_t *pdata, uint8_t datalen )
{
  Wire.beginTransmission(_addr);
  Wire.write(Reg);
  for (uint8_t i = 0; i < datalen; i++){
    Wire.write(pdata[i]);
  }
  Wire.endTransmission();
}

int16_t DFRobot_URM09::scanDevice()
{
  Wire.begin();
  uint8_t error ,address;
  for (address = 1; address < 127; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      return address;
    }
  }
  return -1;
}

void DFRobot_URM09::modifyI2CAddress(uint8_t Address)
{
  txbuf[0] = Address;
  i2cWriteTemDistance(eSLAVEADDR_INDEX ,&txbuf[0] ,1); 
}

uint8_t DFRobot_URM09::getI2CAddress()
{
  uint8_t i = 0;
  uint8_t rxbuf[10]={0};
  Wire.beginTransmission(_addr);
  Wire.write(eSLAVEADDR_INDEX);
  Wire.endTransmission();
  Wire.requestFrom(_addr, (uint8_t)1);
  while (Wire.available()){
    rxbuf[i++] = Wire.read();
  }
  return rxbuf[0];
}