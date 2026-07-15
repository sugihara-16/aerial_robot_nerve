#pragma once

#include <spinal_msgs/msg/spi_link_state.h>

#include <ros_utils/ros_module_base.hpp>

#include "communication/spi_master_link.h"

namespace plexus_link
{

class SpiLinkRosModule final : public RosModuleBase
{
public:
  SpiLinkRosModule()
  : RosModuleBase(
      RosModuleEntityCapacity()
        .max_subscriptions(0)
        .max_publishers(1)
        .max_services(0)
        .max_timers(0))
  {}

  void init_hw(SpiMasterLink* link) { link_ = link; }
  void create_entities(rcl_node_t& node) override;
  void publish() override;

private:
  static constexpr uint32_t kPublishPeriodMs = 20U;
  static constexpr uint32_t kLinkActiveTimeoutMs = 100U;

  SpiMasterLink* link_{nullptr};
  rcl_publisher_t publisher_{};
  spinal_msgs__msg__SpiLinkState message_{};
  ModuleStatePayload state_snapshot_{};
  SpiLinkDiagnostics diagnostics_snapshot_{};
  uint32_t last_publish_tick_ms_{0U};
  uint8_t next_module_index_{0U};
};

}  // namespace plexus_link
