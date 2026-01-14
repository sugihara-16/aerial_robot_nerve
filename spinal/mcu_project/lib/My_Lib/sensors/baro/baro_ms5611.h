/*
******************************************************************************
* File Name          : baro_ms5611.h
* Description        : Baro MS5611 Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif
#ifndef __BARO_MS5611_H
#define __BARO_MS5611_H

#include "sensors/baro/baro.h"
#include <stdint.h>

class Baro : public BaroBackend
{
public:
  Baro();

  void update(bool calibrate = false);
  void accumulate();

  void init_hw(I2C_HandleTypeDef* hi2c,
               GPIO_TypeDef* baro_ctrl_port, uint16_t baro_ctrl_pin);

  static const uint8_t MS561101BA_ADDRESS = 0xEC;

  static const uint8_t CMD_MS56XX_RESET = 0x1E;
  static const uint8_t CMD_MS56XX_READ_ADC = 0x00;

  static const uint8_t CMD_MS56XX_PROM = 0xA0;

  static const uint8_t ADDR_CMD_CONVERT_D1_OSR256  = 0x40;
  static const uint8_t ADDR_CMD_CONVERT_D1_OSR512  = 0x42;
  static const uint8_t ADDR_CMD_CONVERT_D1_OSR1024 = 0x44;
  static const uint8_t ADDR_CMD_CONVERT_D1_OSR2048 = 0x46;
  static const uint8_t ADDR_CMD_CONVERT_D1_OSR4096 = 0x48;

  static const uint8_t ADDR_CMD_CONVERT_D2_OSR256  = 0x50;
  static const uint8_t ADDR_CMD_CONVERT_D2_OSR512  = 0x52;
  static const uint8_t ADDR_CMD_CONVERT_D2_OSR1024 = 0x54;
  static const uint8_t ADDR_CMD_CONVERT_D2_OSR2048 = 0x56;
  static const uint8_t ADDR_CMD_CONVERT_D2_OSR4096 = 0x58;

  static const uint8_t ADDR_CMD_CONVERT_PRESSURE     = ADDR_CMD_CONVERT_D1_OSR4096;
  static const uint8_t ADDR_CMD_CONVERT_TEMPERATURE  = ADDR_CMD_CONVERT_D2_OSR4096;

private:
  void calculate();
  bool readCalib(uint16_t prom[8]);

  uint16_t readCalibWord(uint8_t word);
  uint32_t readAdc();

  GPIO_TypeDef* baro_ctrl_port_{nullptr};
  uint16_t baro_ctrl_pin_{0};

  uint32_t test_value{0};

  volatile bool     tp_updated_{false};
  volatile uint8_t  d1_count_{0};
  volatile uint8_t  d2_count_{0};
  volatile uint32_t s_d1_{0}, s_d2_{0};
  uint8_t           state_{0};

  uint16_t c1_{0}, c2_{0}, c3_{0}, c4_{0}, c5_{0}, c6_{0};
  float    d1_{0.0f}, d2_{0.0f};
};

#endif
