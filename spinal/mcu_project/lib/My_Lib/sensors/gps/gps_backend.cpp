#include "sensors/gps/gps_backend.h"
#include <rmw_microros/rmw_microros.h>

namespace
{
#ifdef STM32H7
  uint8_t rx_buf_[GPS_BUFFER_SIZE] __attribute__((section(".GpsRxBufferSection")));
#else
  uint8_t rx_buf_[GPS_BUFFER_SIZE];
#endif
  uint32_t rd_ptr_ = 0;
}
  
void GPS_Backend::init(UART_HandleTypeDef* huart)
{
  huart_ =huart;
  // use DMA for UART RX
   __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
   __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
   memset(rx_buf_, 0, RX_BUFFER_SIZE);
   HAL_UART_Receive_DMA(huart, rx_buf_, RX_BUFFER_SIZE);
}

bool GPS_Backend::available()
{
  uint32_t dma_write_ptr =  (GPS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (GPS_BUFFER_SIZE);
  return (rd_ptr_ != dma_write_ptr);
}

int GPS_Backend::read()
{
  /* handle RX Overrun Error */
  if ( __HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE) )
    {
      __HAL_UART_CLEAR_FLAG(huart_,
                            UART_CLEAR_NEF | UART_CLEAR_OREF);
      HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE); // restart
    }

  uint32_t dma_write_ptr =  (GPS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (GPS_BUFFER_SIZE);
  int c = -1;
  if(rd_ptr_ != dma_write_ptr)
    {
      c = (int)rx_buf_[rd_ptr_++];
      rd_ptr_ %= GPS_BUFFER_SIZE;
    }
  return c;
}

void GPS_Backend::publish() {
  if (publish_fn_) publish_fn_(publish_ctx_);
}
