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
  if (!link_->snapshot(state_snapshot_, joint_snapshot_, diagnostics_snapshot_)) return;
  last_publish_tick_ms_ = now;

  const uint32_t valid_frame_age = diagnostics_snapshot_.valid_frames == 0U
    ? UINT32_MAX
    : now - diagnostics_snapshot_.last_valid_frame_tick_ms;
  const uint64_t epoch_ms = rmw_uros_epoch_millis();

  message_.stamp.sec = static_cast<int32_t>(epoch_ms / 1000ULL);
  message_.stamp.nanosec = static_cast<uint32_t>((epoch_ms % 1000ULL) * 1000000ULL);
  message_.link_active = diagnostics_snapshot_.valid_frames > 0U &&
                         valid_frame_age <= kLinkActiveTimeoutMs;
  message_.local_node_id = diagnostics_snapshot_.local_node_id;
  message_.plexus_node_id = diagnostics_snapshot_.plexus_node_id;
  message_.node_identity_valid = message_.link_active &&
    isValidNodeId(message_.local_node_id) &&
    message_.plexus_node_id == message_.local_node_id;
  uint8_t active_node_count = 0U;
  if (message_.link_active)
    {
      for (size_t module_index = kFirstRemoteNodeSlot;
           module_index < kModuleCount;
           ++module_index)
        {
          NetworkTestState candidate_state{};
          if (decodeNetworkTestModuleState(
                state_snapshot_.modules[module_index], candidate_state))
            {
              ++active_node_count;
            }
        }
    }

  uint8_t selected_module_index = 0U;
  if (active_node_count > 0U)
    {
      const uint8_t first_candidate =
        next_module_index_ >= kFirstRemoteNodeSlot && next_module_index_ < kModuleCount
        ? next_module_index_
        : static_cast<uint8_t>(kFirstRemoteNodeSlot);
      for (size_t offset = 0U; offset < kRemoteNodeCapacity; ++offset)
        {
          const uint8_t candidate = static_cast<uint8_t>(
            kFirstRemoteNodeSlot +
            ((first_candidate - kFirstRemoteNodeSlot + offset) % kRemoteNodeCapacity));
          NetworkTestState candidate_state{};
          if (decodeNetworkTestModuleState(state_snapshot_.modules[candidate], candidate_state))
            {
              selected_module_index = candidate;
              break;
            }
        }
    }

  message_.module_count = static_cast<uint8_t>(kModuleCount);
  message_.module_index = selected_module_index;
  message_.active_rs485_node_count = active_node_count;
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
  message_.node_identity_mismatches = diagnostics_snapshot_.node_identity_mismatches;

  const ModuleState& state = state_snapshot_.modules[selected_module_index];
  NetworkTestState test_state{};
  message_.rs485_test_valid = selected_module_index >= kFirstRemoteNodeSlot &&
    decodeNetworkTestModuleState(state, test_state);
  message_.remote_imu_valid = message_.rs485_test_valid &&
    test_state.spinal_imu.valid != 0U;
  message_.remote_imu_timestamp_ms = message_.remote_imu_valid
    ? test_state.spinal_imu.timestamp_ms
    : 0U;
  const RemoteJointCacheEntry* joint_entry = nullptr;
  if (message_.rs485_test_valid)
    {
      for (size_t index = 0U; index < kRemoteNodeCapacity; ++index)
        {
          if (joint_snapshot_.entries[index].node_id ==
              test_state.responder_node_id)
            {
              joint_entry = &joint_snapshot_.entries[index];
              break;
            }
        }
    }
  message_.remote_joint_valid =
    joint_entry != nullptr &&
    joint_entry->sample.valid != 0U &&
    static_cast<uint32_t>(now - joint_entry->received_tick_ms) <=
      kLinkActiveTimeoutMs;
  message_.remote_joint_timestamp_ms = message_.remote_joint_valid
    ? joint_entry->sample.timestamp_ms
    : 0U;
  message_.remote_joint_count = message_.remote_joint_valid
    ? joint_entry->sample.count
    : 0U;
  std::memset(
    message_.remote_joint_position_raw,
    0,
    sizeof(message_.remote_joint_position_raw));
  std::memset(
    message_.remote_joint_load_raw,
    0,
    sizeof(message_.remote_joint_load_raw));
  std::memset(message_.joint_position, 0, sizeof(message_.joint_position));
  std::memset(message_.joint_velocity, 0, sizeof(message_.joint_velocity));
  std::memset(message_.joint_torque, 0, sizeof(message_.joint_torque));
  if (message_.remote_joint_valid)
    {
      for (size_t index = 0U; index < message_.remote_joint_count; ++index)
        {
          const int16_t position = joint_entry->sample.position[index];
          const int16_t load = joint_entry->sample.load[index];
          message_.remote_joint_position_raw[index] = position;
          message_.remote_joint_load_raw[index] = load;
          message_.joint_position[index] = static_cast<float>(position);
          message_.joint_torque[index] = static_cast<float>(load);
        }
    }
  if (message_.rs485_test_valid)
    {
      message_.rs485_node_slot = static_cast<uint8_t>(test_state.node_slot);
      message_.rs485_port_index = static_cast<uint8_t>(test_state.root_port_index);
      message_.rs485_physical_port = static_cast<uint8_t>(test_state.root_physical_port);
      message_.rs485_transaction_id = test_state.transaction_id;
      message_.rs485_input_value = test_state.input_value;
      message_.rs485_responder_node_id = test_state.responder_node_id;
      message_.rs485_result_value = test_state.result_value;
      message_.rs485_parent_node_id = test_state.parent_node_id;
      message_.rs485_parent_physical_port =
        static_cast<uint8_t>(test_state.parent_physical_port);
      message_.rs485_upstream_physical_port =
        static_cast<uint8_t>(test_state.upstream_physical_port);
      message_.rs485_hop_count = static_cast<uint8_t>(test_state.hop_count);
    }
  else
    {
      message_.rs485_node_slot = UINT8_MAX;
      message_.rs485_port_index = UINT8_MAX;
      message_.rs485_physical_port = 0U;
      message_.rs485_transaction_id = 0U;
      message_.rs485_input_value = 0U;
      message_.rs485_responder_node_id = 0U;
      message_.rs485_result_value = 0U;
      message_.rs485_parent_node_id = 0U;
      message_.rs485_parent_physical_port = 0U;
      message_.rs485_upstream_physical_port = 0U;
      message_.rs485_hop_count = 0U;
    }

  std::memcpy(message_.acceleration, state.imu.acceleration, sizeof(message_.acceleration));
  std::memcpy(message_.angular_velocity, state.imu.angular_velocity,
              sizeof(message_.angular_velocity));
  std::memcpy(message_.magnetic_field, state.imu.magnetic_field,
              sizeof(message_.magnetic_field));
  std::memcpy(message_.quaternion, state.imu.quaternion, sizeof(message_.quaternion));
  message_.battery_voltage = state.battery_voltage;

  (void)rcl_publish(&publisher_, &message_, nullptr);
  next_module_index_ = message_.rs485_test_valid
    ? static_cast<uint8_t>(
        kFirstRemoteNodeSlot +
        ((selected_module_index - kFirstRemoteNodeSlot + 1U) % kRemoteNodeCapacity))
    : static_cast<uint8_t>(kFirstRemoteNodeSlot);
}

}  // namespace plexus_link
