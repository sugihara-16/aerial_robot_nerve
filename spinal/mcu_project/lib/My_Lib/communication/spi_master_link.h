#pragma once

#include <cstdint>

#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

#include "communication/spi_link_protocol.h"

namespace plexus_link
{

struct SpiLinkDiagnostics
{
  uint32_t attempted_transfers{0U};
  uint32_t completed_transfers{0U};
  uint32_t valid_frames{0U};
  uint32_t invalid_frames{0U};
  uint32_t semantic_errors{0U};
  uint32_t cycle_mismatches{0U};
  uint32_t dma_start_errors{0U};
  uint32_t timeouts{0U};
  uint32_t peripheral_errors{0U};
  uint32_t last_received_cycle{0U};
  uint32_t last_received_timestamp_ms{0U};
  uint32_t last_valid_frame_tick_ms{0U};
};

class SpiMasterLink
{
public:
  void init(SPI_HandleTypeDef* hspi,
            GPIO_TypeDef* chip_select_port,
            uint16_t chip_select_pin,
            osMutexId bus_mutex,
            osMutexId state_mutex,
            osSemaphoreId completion_semaphore);
  void run();

  void notifyTransferCompleteFromIsr();
  void notifyTransferErrorFromIsr();

  bool snapshot(ModuleStatePayload& states, SpiLinkDiagnostics& diagnostics) const;

private:
  enum class TransferEvent : uint8_t
  {
    Waiting,
    Completed,
    Error,
  };

  void executeCycle();
  void setBaudPrescaler(uint32_t prescaler);
  void drainCompletionSemaphore();

  SPI_HandleTypeDef* hspi_{nullptr};
  GPIO_TypeDef* chip_select_port_{nullptr};
  uint16_t chip_select_pin_{0U};
  osMutexId bus_mutex_{nullptr};
  osMutexId state_mutex_{nullptr};
  osSemaphoreId completion_semaphore_{nullptr};
  volatile TransferEvent transfer_event_{TransferEvent::Waiting};
  uint32_t cycle_counter_{0U};

  alignas(32) Frame tx_frame_{};
  alignas(32) Frame rx_frame_{};
  ModuleCommandPayload command_payload_{};
  ModuleStatePayload decoded_states_{};
  ModuleStatePayload latest_states_{};
  SpiLinkDiagnostics diagnostics_{};
};

}  // namespace plexus_link
