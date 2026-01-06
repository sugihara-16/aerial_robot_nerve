#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and register micro-ROS custom transport for STM32.
 *
 * This function must be called once before any rcl/rclc initialization.
 * It registers UART-based custom transport using STM32Hardware.
 */
void microros_transport_init(void);

#ifdef __cplusplus
}
#endif
