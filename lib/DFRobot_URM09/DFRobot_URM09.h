/*!
 * @file DFRobot_URM09.h
 * @brief Basic structure of DFRobot_URM09 class
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author ZhixinLiu(zhixin.liu@dfrobot.com)
 * @version V1.2
 * @date 2021-09-30
 * @url https://github.com/DFRobot/DFRobot_URM09
 */
#ifndef __DFRobot_URM09_H__
#define __DFRobot_URM09_H__

#include <Wire.h>
#include <Arduino.h>
#include <utility>

#define    MEASURE_MODE_AUTOMATIC  0x80           ///< automatic mode
#define    MEASURE_MODE_PASSIVE    0x00           ///< passive mode

#define    CMD_DISTANCE_MEASURE    0x01           ///< passive mode configure registers

#define    MEASURE_RANG_500        0x20           ///< Ranging from 500 
#define    MEASURE_RANG_300        0x10           ///< Ranging from 300 
#define    MEASURE_RANG_150        0x00           ///< Ranging from 150 

class DFRobot_URM09{  
public:

  /**
   * @enum eRegister_t
   * @brief Enum register configuration
   */
  typedef enum{
    eSLAVEADDR_INDEX = 0,
    ePID_INDEX,
    eVERSION_INDEX,
    eDIST_H_INDEX,         /**< High distance eight digits */
    eDIST_L_INDEX,         /**< Low  distance eight digits */
    eTEMP_H_INDEX,         /**< High temperature eight digits */ 
    eTEMP_L_INDEX,         /**< Low  temperature eight digits */
    eCFG_INDEX,
    eCMD_INDEX,
    eREG_NUM
  }eRegister_t;
  
  /**
   * @fn DFRobot_URM09
   * @brief Constructor
   * @return None
   */
  DFRobot_URM09();

  /**
   * @fn DFRobot_URM09
   * @brief Destructor
   * @return None
   */
  ~DFRobot_URM09();

  /**
   * @fn begin
   * @brief Init i2c
   * @param address i2c device number (1-127), default to 0x11 when no device number is uploaded
   * @return Return bool type, indicating init status
   * @retval true Init succeed
   * @retval false init failed
   */
  bool begin(uint8_t address = 0x11);

  /**
   * @fn setModeRange
   * @brief Set mode and measure distance 
   * @param range is measured range
   * @n     MEASURE_RANG_500    Ranging from 500 
   * @n     MEASURE_RANG_300    Ranging from 300 
   * @n     MEASURE_RANG_150    Ranging from 150 
   * @param mode is measurement mode, automatic measurement and passive measurement. 
   * @n     MEASURE_MODE_AUTOMATIC    automatic mode
   * @n     MEASURE_MODE_PASSIVE      passive mode
   * @return None
   */
  void setModeRange(uint8_t range, uint8_t mode);

  /**
   * @fn measurement
   * @brief Measure distance, send measurement command in passive mode.
   * @return None
   */
  void measurement(void);

  /**
   * @fn getTemperature
   * @brief Get temperature value
   * @return Temperature value, unit (℃)
   */
  float getTemperature(void);

  /**
   * @fn getDistance
   * @brief Read distance
   * @return Distance value, unit (cm)
   * @n The increase is 10 times as the actual temperature
   * @n For example: if the read value is 0x00fe, the actual temperature will be 0x00fe / 10 = 25.4
   * @return None
   */
  std::pair<int16_t , uint32_t> getDistance(void);

  /**
   * @fn scanDevice
   * @brief Scan i2c device
   * @return Scanning status
   * @retval -1 Device is not scanned
   * @retval other iic address
   */
  int16_t scanDevice(void);
  
  /**
   * @fn getI2CAddress
   * @brief Get i2c device address number
   * @return i2c device address number
   */
  uint8_t getI2CAddress(void);

  /**
   * @fn modifyI2CAddress
   * @brief Change i2c device address
   * @param address i2c device address number (1-127)
   * @return None
   */
  void modifyI2CAddress(uint8_t address);
  uint8_t txbuf[10] = {0};
private:
  void    i2cWriteTemDistance(uint8_t reg, uint8_t *pdata, uint8_t datalen);
  uint8_t _addr;
  uint32_t start{0} , end{0};
  uint32_t last_mesurment{0};

  uint8_t rxbuf[10] = {0};
  uint8_t process{0};
};
#endif