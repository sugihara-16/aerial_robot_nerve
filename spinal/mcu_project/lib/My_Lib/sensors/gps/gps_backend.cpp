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

  GPS_Backend* gps_instance_ = nullptr;

  void gpsConfigCallbackStatic(const void * msgin)
  {
    if (gps_instance_ == nullptr) return;
    if (msgin == nullptr) return;
    gps_instance_->gpsConfigCallback(*reinterpret_cast<const std_msgs__msg__UInt8*>(msgin));
  }
}

void GPS_Backend::init(UART_HandleTypeDef* huart, rcl_node_t* node, rclc_executor_t* executor)
{
  huart_ =huart;
  node_ = node;
  executor_ = executor;

  gps_instance_ = this;

  rclc_subscription_init_default(
    &gps_config_sub_,
    node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "gps_config_cmd");

  rclc_executor_add_subscription(
    executor_,
    &gps_config_sub_,
    &gps_config_msg_,
    &gpsConfigCallbackStatic,
    ON_NEW_DATA);

  rclc_publisher_init_default(
    &gps_pub_,
    node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, Gps),
    "gps");

  // use DMA for UART RX
   __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
   __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
   HAL_UART_Receive_DMA(huart, rx_buf_, RX_BUFFER_SIZE);

   memset(rx_buf_, 0, RX_BUFFER_SIZE);
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
                            UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
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


void GPS_Backend::publish()
{
#if 0
  gps_full_msg_.stamp = nh_->now();
  gps_full_msg_.status = getGpsState().status; // fix status
  gps_full_msg_.year = getGpsState().utc_year;
  gps_full_msg_.month = getGpsState().utc_month;
  gps_full_msg_.hour = getGpsState().utc_hour;
  gps_full_msg_.day = getGpsState().utc_day;
  gps_full_msg_.min = getGpsState().utc_min;
  gps_full_msg_.sec = getGpsState().utc_sec;
  gps_full_msg_.nano = getGpsState().utc_nano;
  gps_full_msg_.time_valid = getGpsState().utc_valid;

  gps_full_msg_.location[0] = getGpsState().location.lat / 1e7L; // lat
  gps_full_msg_.location[1] = getGpsState().location.lng / 1e7L; // lng
  gps_full_msg_.h_acc = getGpsState().horizontal_accuracy; // lng

  gps_full_msg_.velocity[0] = getGpsState().velocity.x;
  gps_full_msg_.velocity[1] = getGpsState().velocity.y;
  gps_full_msg_.v_acc = getGpsState().speed_accuracy; // lng

  gps_full_msg_.sat_num = getGpsState().num_sats;

  gps_full_pub_.publish(&gps_full_msg_); // reserve
#endif

  {
    uint64_t t_ms = rmw_uros_epoch_millis();
    gps_msg_.stamp.sec = (int32_t)(t_ms / 1000ULL);
    gps_msg_.stamp.nanosec = (uint32_t)((t_ms % 1000ULL) * 1000000ULL);
  }
  gps_msg_.location[0] = getGpsState().location.lat / 1e7L; // lat
  gps_msg_.location[1] = getGpsState().location.lng / 1e7L; // lng
  gps_msg_.velocity[0] = getGpsState().velocity.x;
  gps_msg_.velocity[1] = getGpsState().velocity.y;
  gps_msg_.sat_num = getGpsState().num_sats;

  rcl_publish(&gps_pub_, &gps_msg_, NULL);
}
