#pragma once

#include <atomic>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "cmsis_os.h"

struct RosContext
{
  rcl_allocator_t allocator{};
  rclc_support_t support{};
  rcl_node_t node{};
  rclc_executor_t executor{};

  osMutexId ros_mutex{nullptr};

  std::atomic<bool> ready{false};
};
