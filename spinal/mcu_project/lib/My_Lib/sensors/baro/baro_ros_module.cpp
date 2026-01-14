#include "sensors/baro/baro_ros_module.h"

BaroRosModule* BaroRosModule::self_ = nullptr;

void BaroRosModule::baroConfigCallbackStatic(const void* msgin)
{
  if (!self_) return;
  if (!msgin) return;

  const auto* m = reinterpret_cast<const std_msgs__msg__UInt8*>(msgin);
  if (m->data == 1) {
    // TODO
  }
}

void BaroRosModule::create_entities(rcl_node_t& node)
{
  self_ = this;

  reserve_entities();

  (void)init_subscription_default(
    node,
    baro_config_sub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "baro_config_cmd",
    &baro_config_msg_,
    &BaroRosModule::baroConfigCallbackStatic,
    ON_NEW_DATA);
}
