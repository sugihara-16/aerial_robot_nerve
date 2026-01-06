// posix_time_stub.cpp
#include <stdint.h>
#include <time.h>

// HAL_GetTick() is provided by STM32 HAL (C symbol)
extern "C" uint32_t HAL_GetTick(void);

/*
 * Minimal clock_gettime() implementation for bare-metal/newlib builds.
 * This function is required by micro-ROS (rcutils, microxrcedds).
 *
 * Time is based on HAL_GetTick() (milliseconds since boot).
 * CLOCK_REALTIME and CLOCK_MONOTONIC are treated identically.
 */
extern "C" int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
  (void)clk_id;

  if (tp == nullptr) {
    return -1;
  }

  uint32_t ms = HAL_GetTick();
  tp->tv_sec  = static_cast<time_t>(ms / 1000U);
  tp->tv_nsec = static_cast<long>((ms % 1000U) * 1000000UL);

  return 0;
}
