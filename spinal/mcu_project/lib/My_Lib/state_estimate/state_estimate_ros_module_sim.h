#pragma once

#ifdef SIMULATION

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <spinal_msgs/msg/imu.hpp>

#include "state_estimate/state_estimate.h"

class StateEstimateRosModuleSim
{
public:
  StateEstimateRosModuleSim() = default;

  void init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
  {
    node_ = node;

    estimator_.init();

    imu_pub_ = node_->create_publisher<spinal_msgs::msg::Imu>("imu", 1);
    last_imu_pub_time_ms_ = nowMs_();
  }

  void update()
  {
    estimator_.update();
    publishImuIfNeeded_();
  }

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::Imu>> getImuPub() {
    return imu_pub_;
  }

  StateEstimate* getEstimator() {return &estimator_;}

private:
  static constexpr uint8_t IMU_PUB_INTERVAL_MS = 5;

  uint32_t nowMs_() const
  {
    return static_cast<uint32_t>(node_->get_clock()->now().nanoseconds() / 1000000ULL);
  }

  void publishImuIfNeeded_()
  {
    if (!node_ || !imu_pub_) return;
    
    auto* att = estimator_.getAttEstimator();
    if (!att) return;

    const bool updated = att->consumeUpdated();
    if (!updated) return;

    const uint32_t now_ms = nowMs_();
    if (now_ms - last_imu_pub_time_ms_ < IMU_PUB_INTERVAL_MS) return;
    last_imu_pub_time_ms_ = now_ms;

    imu_msg_.stamp = node_->get_clock()->now();

    const ap::Vector3f mag  = att->getMagVec();
    const ap::Vector3f acc  = att->getAccVec();
    const ap::Vector3f gyro = att->getGyroVec();

    imu_msg_.mag[0] = mag.x;   imu_msg_.mag[1] = mag.y;   imu_msg_.mag[2] = mag.z;
    imu_msg_.acc[0] = acc.x;   imu_msg_.acc[1] = acc.y;   imu_msg_.acc[2] = acc.z;
    imu_msg_.gyro[0] = gyro.x; imu_msg_.gyro[1] = gyro.y; imu_msg_.gyro[2] = gyro.z;

    const ap::Quaternion q = att->getQuaternion();
    imu_msg_.quaternion[0] = q[1];
    imu_msg_.quaternion[1] = q[2];
    imu_msg_.quaternion[2] = q[3];
    imu_msg_.quaternion[3] = q[0];

    imu_pub_->publish(imu_msg_);
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  StateEstimate estimator_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::Imu>> imu_pub_;
  spinal_msgs::msg::Imu imu_msg_;

  uint32_t last_imu_pub_time_ms_{0};
};

#endif  // SIMULATION
