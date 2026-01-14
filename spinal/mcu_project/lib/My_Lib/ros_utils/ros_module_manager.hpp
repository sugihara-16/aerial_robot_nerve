#pragma once

#include <cstddef>
#include <array>
#include "ros_module_base.hpp"

template <size_t MaxModules>
class RosModuleManager
{
public:
  bool add(RosModuleBase* m)
  {
    if (!m) return false;
    if (count_ >= MaxModules) return false;
    modules_[count_++] = m;
    return true;
  }

  size_t total_executor_handles() const
  {
    size_t sum = 0;
    for (size_t i = 0; i < count_; ++i) {
      sum += modules_[i]->executor_handles();
    }
    return sum;
  }

  void create_all(rcl_node_t& node)
  {
    for (size_t i = 0; i < count_; ++i) {
      modules_[i]->create_entities(node);
    }
  }

  void add_all(rclc_executor_t& exec)
  {
    for (size_t i = 0; i < count_; ++i) {
      modules_[i]->add_to_executor(exec);
    }
  }

  void destroy_all(rcl_node_t& node)
  {
    for (size_t i = count_; i-- > 0;) {
      modules_[i]->destroy_entities(node);
    }
  }

  void set_ros_mutex_all(osMutexId* ros_mutex_ptr)
  {
    for (size_t i = count_; i-- > 0;) {
      modules_[i]->set_ros_mutex_ptr(ros_mutex_ptr);
    }
  }

  void set_ros_ready_all(std::atomic<bool>* ros_ready)
  {
    for (size_t i = count_; i-- > 0;) {
      modules_[i]->set_ros_ready(ros_ready);
    }
  }

  void update_all()
  {
    for (size_t i = 0; i < count_; ++i) {
      modules_[i]->update();
    }
  }

private:
  std::array<RosModuleBase*, MaxModules> modules_{};
  size_t count_{0};
};
