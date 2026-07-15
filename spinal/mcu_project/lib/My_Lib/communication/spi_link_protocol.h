#pragma once

// Keep these wire definitions synchronized with
// plexus/mcu_project/lib/My_Lib/communication/spi_link_protocol.h.
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace plexus_link
{

constexpr uint32_t kFrameMagic = 0x53584C50UL;
constexpr uint8_t kProtocolVersion = 1U;
constexpr size_t kModuleCount = 9U;
constexpr size_t kJointCount = 8U;
constexpr size_t kThrusterCount = 4U;
constexpr size_t kJointOffsetCount = 4U;
constexpr size_t kFrameSize = 1536U;
constexpr uint32_t kTransferPeriodMs = 2U;

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
static_assert(sizeof(ModuleStatePayload) == 1368U, "Nine state packets must be 1368 bytes");
static_assert(sizeof(ModuleCommandPayload) == 576U, "Nine command packets must be 576 bytes");
static_assert(sizeof(FrameHeader) == 32U, "SPI frame header must be one cache line");
static_assert(sizeof(Frame) == kFrameSize, "SPI frame must be 1536 bytes");
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

inline bool validateVirtualStatePayload(const ModuleStatePayload& payload, uint32_t cycle_counter)
{
  for (size_t module = 0; module < kModuleCount; ++module)
    {
      ModuleState expected{};
      fillVirtualModuleState(expected, module, cycle_counter);
      if (std::memcmp(&payload.modules[module], &expected, sizeof(expected)) != 0)
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
