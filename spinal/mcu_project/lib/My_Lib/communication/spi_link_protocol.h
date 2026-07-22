#pragma once

// Keep these wire definitions synchronized with
// plexus/mcu_project/lib/My_Lib/communication/spi_link_protocol.h.
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace plexus_link
{

constexpr uint32_t kFrameMagic = 0x53584C50UL;
constexpr uint8_t kProtocolVersion = 2U;
constexpr size_t kModuleCount = 9U;
constexpr size_t kJointCount = 8U;
constexpr size_t kThrusterCount = 4U;
constexpr size_t kJointOffsetCount = 4U;
constexpr size_t kFrameSize = 1536U;
constexpr uint32_t kTransferPeriodMs = 2U;
constexpr size_t kFirstRemoteNodeSlot = 1U;
constexpr size_t kRs485PortCount = 4U;
constexpr size_t kRemoteNodeCapacity = kModuleCount - kFirstRemoteNodeSlot;
constexpr float kNetworkTestStateMarker = 4852.0F;

enum class FrameDirection : uint8_t
{
  SpinalToPlexus = 1U,
  PlexusToSpinal = 2U,
};

struct ImuState
{
  float acceleration[3];
  float angular_velocity[3];
  float magnetic_field[3];
  float quaternion[4];
};

struct ModuleState
{
  ImuState imu;
  float joint_position[kJointCount];
  float joint_velocity[kJointCount];
  float joint_torque[kJointCount];
  float battery_voltage;
};

struct ModuleCommand
{
  float target_thrust[kThrusterCount];
  float target_joint_angle[kJointCount];
  float target_joint_offset[kJointOffsetCount];
};

struct NetworkTestState
{
  uint32_t transaction_id;
  uint32_t input_value;
  uint32_t responder_node_id;
  uint32_t result_value;
  uint32_t node_slot;
  uint32_t root_port_index;
  uint32_t root_physical_port;
  uint32_t parent_node_id;
  uint32_t parent_physical_port;
  uint32_t hop_count;
};

struct ModuleStatePayload
{
  ModuleState modules[kModuleCount];
};

struct ModuleCommandPayload
{
  ModuleCommand modules[kModuleCount];
};

struct FrameHeader
{
  uint32_t magic;
  uint8_t version;
  uint8_t direction;
  uint16_t header_size;
  uint32_t cycle_counter;
  uint16_t payload_size;
  uint8_t module_count;
  uint8_t flags;
  uint32_t timestamp_ms;
  uint32_t payload_crc32;
  uint32_t header_crc32;
  uint32_t reserved;
};

constexpr size_t kFramePayloadCapacity = kFrameSize - sizeof(FrameHeader);

struct Frame
{
  FrameHeader header;
  uint8_t payload[kFramePayloadCapacity];
};

static_assert(sizeof(float) == 4U, "SPI test protocol requires 32-bit float");
static_assert(sizeof(ImuState) == 52U, "IMU state must match the 13-float specification");
static_assert(sizeof(ModuleState) == 152U, "Module state payload must be 152 bytes");
static_assert(sizeof(ModuleCommand) == 64U, "Module command payload must be 64 bytes");
static_assert(sizeof(NetworkTestState) == 40U, "Unexpected network test state size");
static_assert(sizeof(ModuleStatePayload) == 1368U, "Nine state packets must be 1368 bytes");
static_assert(sizeof(ModuleCommandPayload) == 576U, "Nine command packets must be 576 bytes");
static_assert(sizeof(FrameHeader) == 32U, "SPI frame header must be one cache line");
static_assert(sizeof(Frame) == kFrameSize, "SPI frame must be 1536 bytes");
static_assert(kFirstRemoteNodeSlot + kRemoteNodeCapacity == kModuleCount,
              "Remote-node states do not fill the SPI module payload");
static_assert(offsetof(ModuleState, joint_position) == 52U, "Unexpected state field layout");
static_assert(offsetof(ModuleState, joint_velocity) == 84U, "Unexpected state field layout");
static_assert(offsetof(ModuleState, joint_torque) == 116U, "Unexpected state field layout");
static_assert(offsetof(ModuleState, battery_voltage) == 148U, "Unexpected state field layout");
static_assert(offsetof(ModuleCommand, target_joint_angle) == 16U,
              "Unexpected command field layout");
static_assert(offsetof(ModuleCommand, target_joint_offset) == 48U,
              "Unexpected command field layout");
static_assert(offsetof(FrameHeader, cycle_counter) == 8U, "Unexpected header field layout");
static_assert(offsetof(FrameHeader, payload_size) == 12U, "Unexpected header field layout");
static_assert(offsetof(FrameHeader, timestamp_ms) == 16U, "Unexpected header field layout");
static_assert(offsetof(FrameHeader, payload_crc32) == 20U, "Unexpected header field layout");
static_assert(offsetof(FrameHeader, header_crc32) == 24U, "Unexpected header field layout");
static_assert(offsetof(FrameHeader, reserved) == 28U, "Unexpected header field layout");

inline uint32_t crc32(const void* data, size_t length)
{
  static constexpr uint32_t kNibbleTable[16] = {
    0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
    0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
    0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
    0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL,
  };

  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i)
    {
      crc ^= bytes[i];
      crc = (crc >> 4U) ^ kNibbleTable[crc & 0x0FUL];
      crc = (crc >> 4U) ^ kNibbleTable[crc & 0x0FUL];
    }
  return crc ^ 0xFFFFFFFFUL;
}

template<typename Payload>
inline void buildFrame(Frame& frame, FrameDirection direction, uint32_t cycle_counter,
                       uint32_t timestamp_ms, const Payload& payload)
{
  static_assert(sizeof(Payload) <= kFramePayloadCapacity, "Payload does not fit in SPI frame");
  std::memset(&frame, 0, sizeof(frame));
  std::memcpy(frame.payload, &payload, sizeof(payload));
  frame.header.magic = kFrameMagic;
  frame.header.version = kProtocolVersion;
  frame.header.direction = static_cast<uint8_t>(direction);
  frame.header.header_size = static_cast<uint16_t>(sizeof(FrameHeader));
  frame.header.cycle_counter = cycle_counter;
  frame.header.payload_size = static_cast<uint16_t>(sizeof(Payload));
  frame.header.module_count = static_cast<uint8_t>(kModuleCount);
  frame.header.flags = 0U;
  frame.header.timestamp_ms = timestamp_ms;
  frame.header.payload_crc32 = crc32(frame.payload, sizeof(payload));
  frame.header.header_crc32 = crc32(&frame.header, offsetof(FrameHeader, header_crc32));
}

inline bool validateFrame(const Frame& frame, FrameDirection expected_direction,
                          size_t expected_payload_size)
{
  if (frame.header.magic != kFrameMagic || frame.header.version != kProtocolVersion ||
      frame.header.direction != static_cast<uint8_t>(expected_direction) ||
      frame.header.header_size != sizeof(FrameHeader) ||
      frame.header.payload_size != expected_payload_size ||
      frame.header.module_count != kModuleCount || frame.header.reserved != 0U)
    {
      return false;
    }
  const uint32_t header_crc = crc32(&frame.header, offsetof(FrameHeader, header_crc32));
  if (header_crc != frame.header.header_crc32)
    {
      return false;
    }
  return crc32(frame.payload, frame.header.payload_size) == frame.header.payload_crc32;
}

template<typename Payload>
inline bool decodeFrame(const Frame& frame, FrameDirection expected_direction, Payload& payload)
{
  if (!validateFrame(frame, expected_direction, sizeof(Payload)))
    {
      return false;
    }
  std::memcpy(&payload, frame.payload, sizeof(payload));
  return true;
}

inline void fillVirtualModuleState(ModuleState& state, size_t module_index, uint32_t cycle_counter)
{
  const float module = static_cast<float>(module_index + 1U);
  const float cycle = static_cast<float>(cycle_counter % 1000U) * 0.001f;
  for (size_t axis = 0; axis < 3U; ++axis)
    {
      const float axis_value = static_cast<float>(axis);
      state.imu.acceleration[axis] = module + axis_value * 0.1f + cycle;
      state.imu.angular_velocity[axis] = module * 0.1f + axis_value * 0.01f + cycle;
      state.imu.magnetic_field[axis] = module * 0.01f + axis_value * 0.001f + cycle;
    }
  state.imu.quaternion[0] = 0.0f;
  state.imu.quaternion[1] = 0.0f;
  state.imu.quaternion[2] = static_cast<float>(cycle_counter % 360U) * 0.001f;
  state.imu.quaternion[3] = 1.0f;
  for (size_t joint = 0; joint < kJointCount; ++joint)
    {
      const float joint_value = static_cast<float>(joint);
      state.joint_position[joint] = module + joint_value * 0.01f + cycle;
      state.joint_velocity[joint] = module * 0.1f + joint_value * 0.001f + cycle;
      state.joint_torque[joint] = module * 0.2f + joint_value * 0.002f + cycle;
    }
  state.battery_voltage = 25.2f - module * 0.1f -
                          static_cast<float>(cycle_counter % 100U) * 0.001f;
}

inline void fillVirtualStatePayload(ModuleStatePayload& payload, uint32_t cycle_counter)
{
  for (size_t module = 0; module < kModuleCount; ++module)
    {
      fillVirtualModuleState(payload.modules[module], module, cycle_counter);
    }
}

inline float encodeNetworkTestHalfWord(uint32_t value, uint32_t shift)
{
  return static_cast<float>((value >> shift) & 0xFFFFU);
}

inline bool decodeNetworkTestHalfWord(float value, uint32_t& half_word)
{
  if (value < 0.0F || value > 65535.0F)
    {
      return false;
    }
  half_word = static_cast<uint32_t>(value);
  return static_cast<float>(half_word) == value;
}

inline void fillNetworkTestModuleState(ModuleState& state, const NetworkTestState& test_state)
{
  state = {};
  state.imu.acceleration[0] = encodeNetworkTestHalfWord(test_state.input_value, 0U);
  state.imu.acceleration[1] = encodeNetworkTestHalfWord(test_state.input_value, 16U);
  state.imu.acceleration[2] = encodeNetworkTestHalfWord(test_state.result_value, 0U);
  state.imu.angular_velocity[0] = encodeNetworkTestHalfWord(test_state.result_value, 16U);
  state.imu.angular_velocity[1] = encodeNetworkTestHalfWord(test_state.responder_node_id, 0U);
  state.imu.angular_velocity[2] = encodeNetworkTestHalfWord(test_state.responder_node_id, 16U);
  state.imu.magnetic_field[0] = encodeNetworkTestHalfWord(test_state.transaction_id, 0U);
  state.imu.magnetic_field[1] = encodeNetworkTestHalfWord(test_state.transaction_id, 16U);
  state.imu.magnetic_field[2] = kNetworkTestStateMarker;
  state.imu.quaternion[3] = 1.0F;
  state.joint_position[0] = static_cast<float>(test_state.node_slot);
  state.joint_position[1] = static_cast<float>(test_state.root_port_index);
  state.joint_position[2] = static_cast<float>(test_state.root_physical_port);
  state.joint_position[3] = encodeNetworkTestHalfWord(test_state.parent_node_id, 0U);
  state.joint_position[4] = encodeNetworkTestHalfWord(test_state.parent_node_id, 16U);
  state.joint_position[5] = static_cast<float>(test_state.parent_physical_port);
  state.joint_position[6] = static_cast<float>(test_state.hop_count);
}

inline bool decodeNetworkTestModuleState(const ModuleState& state, NetworkTestState& test_state)
{
  if (state.imu.magnetic_field[2] != kNetworkTestStateMarker ||
      state.imu.quaternion[3] != 1.0F)
    {
      return false;
    }

  uint32_t input_low = 0U;
  uint32_t input_high = 0U;
  uint32_t result_low = 0U;
  uint32_t result_high = 0U;
  uint32_t node_low = 0U;
  uint32_t node_high = 0U;
  uint32_t transaction_low = 0U;
  uint32_t transaction_high = 0U;
  uint32_t node_slot = 0U;
  uint32_t root_port_index = 0U;
  uint32_t root_physical_port = 0U;
  uint32_t parent_low = 0U;
  uint32_t parent_high = 0U;
  uint32_t parent_physical_port = 0U;
  uint32_t hop_count = 0U;
  if (!decodeNetworkTestHalfWord(state.imu.acceleration[0], input_low) ||
      !decodeNetworkTestHalfWord(state.imu.acceleration[1], input_high) ||
      !decodeNetworkTestHalfWord(state.imu.acceleration[2], result_low) ||
      !decodeNetworkTestHalfWord(state.imu.angular_velocity[0], result_high) ||
      !decodeNetworkTestHalfWord(state.imu.angular_velocity[1], node_low) ||
      !decodeNetworkTestHalfWord(state.imu.angular_velocity[2], node_high) ||
      !decodeNetworkTestHalfWord(state.imu.magnetic_field[0], transaction_low) ||
      !decodeNetworkTestHalfWord(state.imu.magnetic_field[1], transaction_high) ||
      !decodeNetworkTestHalfWord(state.joint_position[0], node_slot) ||
      !decodeNetworkTestHalfWord(state.joint_position[1], root_port_index) ||
      !decodeNetworkTestHalfWord(state.joint_position[2], root_physical_port) ||
      !decodeNetworkTestHalfWord(state.joint_position[3], parent_low) ||
      !decodeNetworkTestHalfWord(state.joint_position[4], parent_high) ||
      !decodeNetworkTestHalfWord(state.joint_position[5], parent_physical_port) ||
      !decodeNetworkTestHalfWord(state.joint_position[6], hop_count))
    {
      return false;
    }

  test_state.transaction_id = transaction_low | (transaction_high << 16U);
  test_state.input_value = input_low | (input_high << 16U);
  test_state.responder_node_id = node_low | (node_high << 16U);
  test_state.result_value = result_low | (result_high << 16U);
  test_state.node_slot = node_slot;
  test_state.root_port_index = root_port_index;
  test_state.root_physical_port = root_physical_port;
  test_state.parent_node_id = parent_low | (parent_high << 16U);
  test_state.parent_physical_port = parent_physical_port;
  test_state.hop_count = hop_count;
  return test_state.responder_node_id != 0U &&
         test_state.node_slot < kRemoteNodeCapacity &&
         test_state.root_port_index < kRs485PortCount &&
         test_state.root_physical_port >= 2U && test_state.root_physical_port <= 5U &&
         test_state.parent_node_id != 0U &&
         test_state.parent_physical_port >= 2U &&
         test_state.parent_physical_port <= 5U &&
         test_state.hop_count > 0U && test_state.hop_count <= 8U &&
         test_state.result_value ==
           (test_state.input_value ^ test_state.responder_node_id);
}

inline void fillVirtualModuleCommand(ModuleCommand& command, size_t module_index,
                                     uint32_t cycle_counter)
{
  const float module = static_cast<float>(module_index + 1U);
  const float cycle = static_cast<float>(cycle_counter % 1000U) * 0.001f;
  for (size_t thruster = 0; thruster < kThrusterCount; ++thruster)
    {
      command.target_thrust[thruster] = module + static_cast<float>(thruster) * 0.1f + cycle;
    }
  for (size_t joint = 0; joint < kJointCount; ++joint)
    {
      command.target_joint_angle[joint] = module * 0.01f +
                                           static_cast<float>(joint) * 0.05f + cycle;
    }
  for (size_t offset = 0; offset < kJointOffsetCount; ++offset)
    {
      command.target_joint_offset[offset] = module * 0.001f +
                                             static_cast<float>(offset) * 0.01f + cycle;
    }
}

inline void fillVirtualCommandPayload(ModuleCommandPayload& payload, uint32_t cycle_counter)
{
  for (size_t module = 0; module < kModuleCount; ++module)
    {
      fillVirtualModuleCommand(payload.modules[module], module, cycle_counter);
    }
}

inline bool virtualFloatMatches(float actual, float expected)
{
  constexpr float kTolerance = 1.0e-5f;
  const float difference = actual - expected;
  return difference >= -kTolerance && difference <= kTolerance;
}

inline bool virtualModuleStateMatches(const ModuleState& actual,
                                      const ModuleState& expected)
{
  for (size_t axis = 0U; axis < 3U; ++axis)
    {
      if (!virtualFloatMatches(actual.imu.acceleration[axis], expected.imu.acceleration[axis]) ||
          !virtualFloatMatches(
            actual.imu.angular_velocity[axis], expected.imu.angular_velocity[axis]) ||
          !virtualFloatMatches(actual.imu.magnetic_field[axis], expected.imu.magnetic_field[axis]))
        {
          return false;
        }
    }
  for (size_t component = 0U; component < 4U; ++component)
    {
      if (!virtualFloatMatches(
            actual.imu.quaternion[component], expected.imu.quaternion[component]))
        {
          return false;
        }
    }
  for (size_t joint = 0U; joint < kJointCount; ++joint)
    {
      if (!virtualFloatMatches(actual.joint_position[joint], expected.joint_position[joint]) ||
          !virtualFloatMatches(actual.joint_velocity[joint], expected.joint_velocity[joint]) ||
          !virtualFloatMatches(actual.joint_torque[joint], expected.joint_torque[joint]))
        {
          return false;
        }
    }
  return virtualFloatMatches(actual.battery_voltage, expected.battery_voltage);
}

inline bool validateVirtualStatePayload(const ModuleStatePayload& payload, uint32_t cycle_counter)
{
  for (size_t module = 0; module < kModuleCount; ++module)
    {
      NetworkTestState test_state{};
      if (decodeNetworkTestModuleState(payload.modules[module], test_state))
        {
          continue;
        }
      ModuleState expected{};
      fillVirtualModuleState(expected, module, cycle_counter);
      if (!virtualModuleStateMatches(payload.modules[module], expected))
        {
          return false;
        }
    }
  return true;
}

inline bool validateVirtualCommandPayload(const ModuleCommandPayload& payload,
                                          uint32_t cycle_counter)
{
  for (size_t module = 0; module < kModuleCount; ++module)
    {
      ModuleCommand expected{};
      fillVirtualModuleCommand(expected, module, cycle_counter);

      for (size_t thruster = 0; thruster < kThrusterCount; ++thruster)
        {
          if (!virtualFloatMatches(payload.modules[module].target_thrust[thruster],
                                   expected.target_thrust[thruster]))
            {
              return false;
            }
        }
      for (size_t joint = 0; joint < kJointCount; ++joint)
        {
          if (!virtualFloatMatches(payload.modules[module].target_joint_angle[joint],
                                   expected.target_joint_angle[joint]))
            {
              return false;
            }
        }
      for (size_t offset = 0; offset < kJointOffsetCount; ++offset)
        {
          if (!virtualFloatMatches(payload.modules[module].target_joint_offset[offset],
                                   expected.target_joint_offset[offset]))
            {
              return false;
            }
        }
    }
  return true;
}

}  // namespace plexus_link
