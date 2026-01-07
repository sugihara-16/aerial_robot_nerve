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

MagEncoder::MagEncoder():
  angle_pub_()
{
}

void MagEncoder::init(I2C_HandleTypeDef* hi2c, rcl_node_t* node)
{
  hi2c_ = hi2c;
  node_ = node;

  rclc_publisher_init_default(
    &angle_pub_,
    node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    "encoder_angle");

  raw_encoder_value_ = 0;

  last_time_ = HAL_GetTick() + 6000; // after 6s
}

void MagEncoder::update(void)
{
  uint32_t now_time = HAL_GetTick();
  if(now_time >= last_time_ + UPDATE_INTERVAL)
    {
      last_time_ = now_time;
      uint8_t val[1];
      val[0] = AS5600_REG_RAW_ANGLE;
      int i2c_status = HAL_I2C_Master_Transmit(hi2c_, AS5600_I2C_ADDRESS, val, 1, 100);
      if(i2c_status == HAL_OK)
        {
          uint8_t adc[2];
          HAL_I2C_Master_Receive(hi2c_, AS5600_I2C_ADDRESS , adc, 2, 100);
          raw_encoder_value_ = (uint16_t)(adc[0] << 8 | adc[1]);
        }
      else
        {
          raw_encoder_value_ = 65535;
        }
      angle_msg_.data = raw_encoder_value_;
      rcl_publish(&angle_pub_, &angle_msg_, NULL);
    }

}
