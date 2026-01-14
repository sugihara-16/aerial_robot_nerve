#pragma once

#include <std_msgs/msg/u_int16.h>
#include <ros_utils/ros_module_base.hpp>
#include "sensors/encoder/mag_encoder.h"

class EncoderRosModule final : public RosModuleBase
{
public:
  EncoderRosModule() = default;

  void init_hw(I2C_HandleTypeDef* hi2c);

  void create_entities(rcl_node_t& node) override;
  void update() override;

private:
  MagEncoder encoder_;

  rcl_publisher_t angle_pub_{};
  std_msgs__msg__UInt16 angle_msg_{};
};
