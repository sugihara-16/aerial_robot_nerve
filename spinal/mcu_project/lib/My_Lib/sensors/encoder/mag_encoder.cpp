/*
******************************************************************************
* File Name          : mag_encoder.cpp
* Description        : Magnetic Encoder Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "sensors/encoder/mag_encoder.h"

void MagEncoder::init(I2C_HandleTypeDef* hi2c)
{
  hi2c_ = hi2c;
  raw_encoder_value_ = 0;
  updated_ = false;

  last_time_ = HAL_GetTick() + 6000;
}

void MagEncoder::update()
{
  const uint32_t now = HAL_GetTick();
  if (now < last_time_ + UPDATE_INTERVAL_MS) return;
  last_time_ = now;

  uint8_t reg = AS5600_REG_RAW_ANGLE;
  const int tx_ok = HAL_I2C_Master_Transmit(hi2c_, AS5600_I2C_ADDRESS, &reg, 1, 100);

  if (tx_ok == HAL_OK) {
    uint8_t adc[2];
    const int rx_ok = HAL_I2C_Master_Receive(hi2c_, AS5600_I2C_ADDRESS, adc, 2, 100);
    if (rx_ok == HAL_OK) {
      raw_encoder_value_ = (uint16_t)((adc[0] << 8) | adc[1]);
    } else {
      raw_encoder_value_ = 65535;
    }
  } else {
    raw_encoder_value_ = 65535;
  }

  updated_ = true;
}
