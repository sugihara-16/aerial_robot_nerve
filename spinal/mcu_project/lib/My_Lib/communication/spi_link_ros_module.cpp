#include "communication/spi_link_ros_module.h"

#include <cstring>

#include <rmw_microros/rmw_microros.h>

namespace plexus_link
{

void SpiLinkRosModule::create_entities(rcl_node_t& node)
{
  reserve_entities();
  (void)spinal_msgs__msg__SpiLinkState__init(&message_);

  (void)init_publisher_default(
    node,
    publisher_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, SpiLinkState),
    "spi_link_state");
}

void SpiLinkRosModule::publish()
{
  if (link_ == nullptr || ros_ready_ == nullptr) return;
  if (!ros_ready_->load(std::memory_order_acquire)) return;

  const uint32_t now = HAL_GetTick();
  if ((now - last_publish_tick_ms_) < kPublishPeriodMs) return;
  if (!link_->snapshot(state_snapshot_, diagnostics_snapshot_)) return;
  last_publish_tick_ms_ = now;

  const uint32_t valid_frame_age = diagnostics_snapshot_.valid_frames == 0U
    ? UINT32_MAX
    : now - diagnostics_snapshot_.last_valid_frame_tick_ms;
  const uint64_t epoch_ms = rmw_uros_epoch_millis();

  message_.stamp.sec = static_cast<int32_t>(epoch_ms / 1000ULL);
  message_.stamp.nanosec = static_cast<uint32_t>((epoch_ms % 1000ULL) * 1000000ULL);
  message_.link_active = diagnostics_snapshot_.valid_frames > 0U &&
                         valid_frame_age <= kLinkActiveTimeoutMs;
  message_.module_count = static_cast<uint8_t>(kModuleCount);
  message_.module_index = next_module_index_;
  message_.last_received_cycle = diagnostics_snapshot_.last_received_cycle;
  message_.plexus_timestamp_ms = diagnostics_snapshot_.last_received_timestamp_ms;
  message_.last_valid_frame_age_ms = valid_frame_age;
  message_.attempted_transfers = diagnostics_snapshot_.attempted_transfers;
  message_.completed_transfers = diagnostics_snapshot_.completed_transfers;
  message_.valid_frames = diagnostics_snapshot_.valid_frames;
  message_.invalid_frames = diagnostics_snapshot_.invalid_frames;
  message_.semantic_errors = diagnostics_snapshot_.semantic_errors;
  message_.cycle_mismatches = diagnostics_snapshot_.cycle_mismatches;
  message_.dma_start_errors = diagnostics_snapshot_.dma_start_errors;
  message_.timeouts = diagnostics_snapshot_.timeouts;
  message_.peripheral_errors = diagnostics_snapshot_.peripheral_errors;

  const ModuleState& state = state_snapshot_.modules[next_module_index_];
  std::memcpy(message_.acceleration, state.imu.acceleration, sizeof(message_.acceleration));
  std::memcpy(message_.angular_velocity, state.imu.angular_velocity,
              sizeof(message_.angular_velocity));
  std::memcpy(message_.magnetic_field, state.imu.magnetic_field,
              sizeof(message_.magnetic_field));
  std::memcpy(message_.quaternion, state.imu.quaternion, sizeof(message_.quaternion));
  std::memcpy(message_.joint_position, state.joint_position, sizeof(message_.joint_position));
  std::memcpy(message_.joint_velocity, state.joint_velocity, sizeof(message_.joint_velocity));
  std::memcpy(message_.joint_torque, state.joint_torque, sizeof(message_.joint_torque));
  message_.battery_voltage = state.battery_voltage;

  (void)rcl_publish(&publisher_, &message_, nullptr);
  next_module_index_ = static_cast<uint8_t>((next_module_index_ + 1U) % kModuleCount);
}

}  // namespace plexus_link
