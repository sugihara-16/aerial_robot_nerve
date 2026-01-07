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
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <std_msgs/msg/u_int16.h>


class MagEncoder
{
public:
  MagEncoder();
  ~MagEncoder(){};
  
  static const uint8_t AS5600_I2C_ADDRESS =  0x6c; // 0x36 << 1; NOTE: STM: i2c_address << 1 !!!
  static const uint8_t AS5600_REG_RAW_ANGLE =  0x0C;
  static const uint8_t UPDATE_INTERVAL = 20; //20 -> 50Hz

  void init(I2C_HandleTypeDef* hi2c, rcl_node_t* node);
  void update(void);

private:
  I2C_HandleTypeDef* hi2c_;
  rcl_node_t* node_;
  rcl_publisher_t angle_pub_;

  uint16_t raw_encoder_value_;
  uint32_t last_time_;

  std_msgs__msg__UInt16 angle_msg_;

};

#endif
