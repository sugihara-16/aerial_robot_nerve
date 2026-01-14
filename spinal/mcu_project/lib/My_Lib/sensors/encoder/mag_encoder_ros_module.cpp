#include "sensors/encoder/mag_encoder_ros_module.h"
#include <rmw_microros/rmw_microros.h>

void EncoderRosModule::init_hw(I2C_HandleTypeDef* hi2c)
{
  encoder_.init(hi2c);
}

void EncoderRosModule::create_entities(rcl_node_t& node)
{
  reserve_entities();

  (void)init_publisher_default(
    node,
    angle_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    "encoder_angle");
}

void EncoderRosModule::update()
{
  encoder_.update();

  if (!ros_ready_ || !ros_ready_->load()) return;

  if (!encoder_.updated()) return;
  encoder_.clear_updated();

  angle_msg_.data = encoder_.raw_angle();

  lock_ros_();
  (void)rcl_publish(&angle_pub_, &angle_msg_, nullptr);
  unlock_ros_();
}
