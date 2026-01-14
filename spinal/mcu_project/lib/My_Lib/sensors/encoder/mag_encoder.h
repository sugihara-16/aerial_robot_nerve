/*
******************************************************************************
* File Name          : mag_encoder.h
* Description        : Magnetic Encoder Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __MAG_ENCODER_H
#define __MAG_ENCODER_H

#include "config.h"
#include <cstdint>

class MagEncoder
{
public:
  MagEncoder() = default;
  ~MagEncoder() = default;

  static const uint8_t  AS5600_I2C_ADDRESS   = 0x6c; // 0x36<<1 (STM I2C addr shifted)
  static const uint8_t  AS5600_REG_RAW_ANGLE = 0x0C;
  static const uint32_t UPDATE_INTERVAL_MS   = 20;   // 50Hz

  void init(I2C_HandleTypeDef* hi2c);
  void update();

  uint16_t raw_angle() const { return raw_encoder_value_; }
  bool updated() const { return updated_; }
  void clear_updated() { updated_ = false; }

private:
  I2C_HandleTypeDef* hi2c_{nullptr};
  uint16_t raw_encoder_value_{0};
  uint32_t last_time_{0};
  bool updated_{false};
};

#endif
