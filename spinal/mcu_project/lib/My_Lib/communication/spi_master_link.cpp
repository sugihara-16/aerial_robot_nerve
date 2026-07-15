#include "communication/spi_master_link.h"

namespace
{

constexpr uint32_t kImuPrescaler = SPI_BAUDRATEPRESCALER_32;
constexpr uint32_t kPlexusPrescaler = SPI_BAUDRATEPRESCALER_8;
constexpr uint32_t kTransferTimeoutMs = 2U;
constexpr uint32_t kChipSelectSetupUs = 3U;

plexus_link::SpiMasterLink* g_spi_master_link = nullptr;

void cleanDCache(const void* address, size_t size)
{
  SCB_CleanDCache_by_Addr(
    reinterpret_cast<uint32_t*>(const_cast<void*>(address)),
    static_cast<int32_t>(size));
  __DSB();
}

void invalidateDCache(void* address, size_t size)
{
  SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(address), static_cast<int32_t>(size));
  __DSB();
}

void delayMicroseconds(uint32_t microseconds)
{
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
      DWT->CYCCNT = 0U;
      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

  const uint32_t cycles =
    static_cast<uint32_t>((static_cast<uint64_t>(SystemCoreClock) * microseconds) / 1000000ULL);
  const uint32_t start = DWT->CYCCNT;
  while (static_cast<uint32_t>(DWT->CYCCNT - start) < cycles)
    {
      __NOP();
    }
}

}  // namespace

namespace plexus_link
{

void SpiMasterLink::init(SPI_HandleTypeDef* hspi,
                         GPIO_TypeDef* chip_select_port,
                         uint16_t chip_select_pin,
                         osMutexId bus_mutex,
                         osMutexId state_mutex,
                         osSemaphoreId completion_semaphore)
{
  hspi_ = hspi;
  chip_select_port_ = chip_select_port;
  chip_select_pin_ = chip_select_pin;
  bus_mutex_ = bus_mutex;
  state_mutex_ = state_mutex;
  completion_semaphore_ = completion_semaphore;
  HAL_GPIO_WritePin(chip_select_port_, chip_select_pin_, GPIO_PIN_SET);
  drainCompletionSemaphore();
  g_spi_master_link = this;
}

bool SpiMasterLink::snapshot(ModuleStatePayload& states,
                             SpiLinkDiagnostics& diagnostics) const
{
  if (state_mutex_ == nullptr || osMutexWait(state_mutex_, osWaitForever) != osOK)
    {
      return false;
    }

  states = latest_states_;
  diagnostics = diagnostics_;
  (void)osMutexRelease(state_mutex_);
  return true;
}

void SpiMasterLink::setBaudPrescaler(uint32_t prescaler)
{
  __HAL_SPI_DISABLE(hspi_);
  MODIFY_REG(hspi_->Instance->CFG1, SPI_CFG1_MBR, prescaler);
  hspi_->Init.BaudRatePrescaler = prescaler;
}

void SpiMasterLink::drainCompletionSemaphore()
{
  // This project's CMSIS-RTOS v1 wrapper returns osOK (zero) when a token is
  // acquired and a positive osError value when no token is available.
  while (osSemaphoreWait(completion_semaphore_, 0U) == osOK)
    {
    }
}

void SpiMasterLink::executeCycle()
{
  fillVirtualCommandPayload(command_payload_, cycle_counter_);
  buildFrame(tx_frame_, FrameDirection::SpinalToPlexus, cycle_counter_, HAL_GetTick(),
             command_payload_);
  std::memset(&rx_frame_, 0, sizeof(rx_frame_));

  cleanDCache(&tx_frame_, sizeof(tx_frame_));
  invalidateDCache(&rx_frame_, sizeof(rx_frame_));
  drainCompletionSemaphore();

  (void)osMutexWait(bus_mutex_, osWaitForever);
  setBaudPrescaler(kPlexusPrescaler);
  transfer_event_ = TransferEvent::Waiting;
  HAL_GPIO_WritePin(chip_select_port_, chip_select_pin_, GPIO_PIN_RESET);

  // Plexus routes physical CS to PB7, which has no SPI1_NSS alternate
  // function on STM32H743VITx. Give its software-NSS EXTI handler enough time
  // to assert SSI before the first 25 MHz SCK edge.
  delayMicroseconds(kChipSelectSetupUs);

  const HAL_StatusTypeDef start_status = HAL_SPI_TransmitReceive_DMA(
    hspi_,
    reinterpret_cast<uint8_t*>(&tx_frame_),
    reinterpret_cast<uint8_t*>(&rx_frame_),
    static_cast<uint16_t>(sizeof(Frame)));

  bool completed = false;
  bool dma_start_error = false;
  bool timeout = false;
  bool peripheral_error = false;
  bool valid_frame = false;
  bool invalid_frame = false;
  bool semantic_error = false;
  bool cycle_mismatch = false;
  if (start_status == HAL_OK)
    {
      completed = osSemaphoreWait(completion_semaphore_, kTransferTimeoutMs) == osOK &&
                  transfer_event_ == TransferEvent::Completed;
    }
  else
    {
      dma_start_error = true;
    }

  HAL_GPIO_WritePin(chip_select_port_, chip_select_pin_, GPIO_PIN_SET);

  if (!completed)
    {
      if (start_status == HAL_OK && transfer_event_ == TransferEvent::Waiting)
        {
          timeout = true;
        }
      else if (transfer_event_ == TransferEvent::Error)
        {
          peripheral_error = true;
        }
      (void)HAL_SPI_Abort(hspi_);
    }

  setBaudPrescaler(kImuPrescaler);
  (void)osMutexRelease(bus_mutex_);

  if (completed)
    {
      invalidateDCache(&rx_frame_, sizeof(rx_frame_));

      if (!decodeFrame(rx_frame_, FrameDirection::PlexusToSpinal, decoded_states_))
        {
          invalid_frame = true;
        }
      else
        {
          valid_frame = true;

          if (rx_frame_.header.cycle_counter != cycle_counter_)
            {
              cycle_mismatch = true;
            }
          if (!validateVirtualStatePayload(decoded_states_, rx_frame_.header.cycle_counter))
            {
              semantic_error = true;
            }
        }
    }

  (void)osMutexWait(state_mutex_, osWaitForever);
  ++diagnostics_.attempted_transfers;
  diagnostics_.completed_transfers += completed ? 1U : 0U;
  diagnostics_.valid_frames += valid_frame ? 1U : 0U;
  diagnostics_.invalid_frames += invalid_frame ? 1U : 0U;
  diagnostics_.semantic_errors += semantic_error ? 1U : 0U;
  diagnostics_.cycle_mismatches += cycle_mismatch ? 1U : 0U;
  diagnostics_.dma_start_errors += dma_start_error ? 1U : 0U;
  diagnostics_.timeouts += timeout ? 1U : 0U;
  diagnostics_.peripheral_errors += peripheral_error ? 1U : 0U;
  if (valid_frame)
    {
      latest_states_ = decoded_states_;
      diagnostics_.last_received_cycle = rx_frame_.header.cycle_counter;
      diagnostics_.last_received_timestamp_ms = rx_frame_.header.timestamp_ms;
      diagnostics_.last_valid_frame_tick_ms = HAL_GetTick();
    }
  (void)osMutexRelease(state_mutex_);

  ++cycle_counter_;
}

void SpiMasterLink::run()
{
  osDelay(100U);  // Allow the Plexus peripheral task to arm its first DMA transfer.
  uint32_t next_cycle_tick = osKernelSysTick();

  for (;;)
    {
      executeCycle();

      next_cycle_tick += kTransferPeriodMs;
      const uint32_t now = osKernelSysTick();
      const int32_t ticks_until_next_cycle =
        static_cast<int32_t>(next_cycle_tick - now);

      if (ticks_until_next_cycle > 0)
        {
          (void)osDelay(static_cast<uint32_t>(ticks_until_next_cycle));
        }
      else
        {
          // INCLUDE_vTaskDelayUntil is disabled in this CubeMX project.  Always
          // block for at least one tick after an overrun so this high-priority
          // task cannot starve the micro-ROS and sensor tasks.
          next_cycle_tick = now;
          (void)osDelay(1U);
        }
    }
}

void SpiMasterLink::notifyTransferCompleteFromIsr()
{
  transfer_event_ = TransferEvent::Completed;
  (void)osSemaphoreRelease(completion_semaphore_);
}

void SpiMasterLink::notifyTransferErrorFromIsr()
{
  transfer_event_ = TransferEvent::Error;
  (void)osSemaphoreRelease(completion_semaphore_);
}

}  // namespace plexus_link

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (g_spi_master_link != nullptr && hspi->Instance == SPI1)
    {
      g_spi_master_link->notifyTransferCompleteFromIsr();
    }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
  if (g_spi_master_link != nullptr && hspi->Instance == SPI1)
    {
      g_spi_master_link->notifyTransferErrorFromIsr();
    }
}
