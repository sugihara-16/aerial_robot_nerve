/*
******************************************************************************
* File Name          : attitude_estimate.h
* Description        : attitude estimate interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif
#ifndef __ATTITUDE_ESTIMATE_H
#define __ATTITUDE_ESTIMATE_H
#ifndef SIMULATION
#include "config.h"
#include <math/AP_Math.h>

// /* ros */
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rmw_microros/rmw_microros.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#endif

#ifdef SIMULATION
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <spinal_msgs/msg/desire_coord.hpp>
#include <spinal_msgs/msg/imu.hpp>
#else
#include <spinal_msgs/msg/imu.h>
#include <spinal_msgs/srv/mag_declination.h>
#endif

/* sensors */
#ifdef SIMULATION
#include <tf2/LinearMath/Matrix3x3.h>
#else
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"
#include "sensors/imu/drivers/icm20948/icm_20948.h"
#include "sensors/gps/gps_ublox.h"
#endif

/* estiamtor algorithm */
#include "state_estimate/attitude/complementary_ahrs.h"
//#include "state_estimate/attitude/madgwick_ahrs.h"

#include <vector>

#define COMPLEMENTARY 1
#define MADWICK 2
//#define MAHONY 3

/* please change the algorithm type according to your application */
#define ESTIMATE_TYPE COMPLEMENTARY

#ifndef SIMULATION
class AttitudeEstimate;
namespace
{
  extern AttitudeEstimate* attitude_instance_;
  void magDeclinationCallbackStatic(const void * req_msg, void * res_msg);
}
#endif

class AttitudeEstimate {
 public:
  ~AttitudeEstimate() {}
#ifdef SIMULATION
  AttitudeEstimate() : acc_(0, 0, 9.8), mag_(1, 0, 0) {}

  void init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) {
    node_ = node;
    imu_pub_ = node_->create_publisher<spinal_msgs::msg::Imu>("imu", 1);

    last_imu_pub_time_ = HAL_GetTick();
    last_attitude_pub_time_ = HAL_GetTick();
    use_ground_truth_ = false;

#if ESTIMATE_TYPE == COMPLEMENTARY
    estimator_ = new ComplementaryAHRS();
#elif ESTIMATE_TYPE == MADWICK
    estimator_ = new MadgwickAHRS();
#else
#error "no instance for estimator"
#endif
  }

  void update() {
    /* attitude estimation */
    estimator_->update(gyro_, acc_, mag_);
    /* send message to ros*/
    publish();
  }

  void setMag(float x, float y, float z) {
    mag_.x = x;
    mag_.y = y;
    mag_.z = z;
  }
  void setGyro(float x, float y, float z) {
    gyro_.x = x;
    gyro_.y = y;
    gyro_.z = z;
  }
  void setAcc(float x, float y, float z) {
    acc_.x = x;
    acc_.y = y;
    acc_.z = z;
  }

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::Imu>> getImuPub() { return imu_pub_; }

#else
  AttitudeEstimate()
  {}

  void init(IMU* imu, GPS* gps, rcl_node_t* node, rclc_executor_t* executor)
  {
    node_ = node;
    executor_ = executor;

    rclc_publisher_init_default(
      &imu_pub_,
      node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, Imu),
      "imu");

    rclc_service_init_default(
      &mag_declination_srv_,
      node_,
      ROSIDL_GET_SRV_TYPE_SUPPORT(spinal_msgs, srv, MagDeclination),
      "mag_declination");

    rclc_executor_add_service(
      executor_,
      &mag_declination_srv_,
      &mag_declination_req_,
      &mag_declination_res_,
      magDeclinationCallbackStatic);

    attitude_instance_ = this;

    imu_ = imu;
    gps_ = gps;

    last_imu_pub_time_ = HAL_GetTick();
    last_attitude_pub_time_ = HAL_GetTick();
    use_ground_truth_ = false;

#if ESTIMATE_TYPE == COMPLEMENTARY
    estimator_ = new ComplementaryAHRS();
#elif ESTIMATE_TYPE == MADWICK
    estimator_ = new MadgwickAHRS();
#else
#error "no instance for estimator"
#endif
  }

  void update()
  {
    if(gps_)
      {
        if(gps_->getMagValid() && !estimator_->getMagDecValid())
          {
            /* update magnetic declination by GPS receive data */
            estimator_->setMagDeclination(gps_->getMagDeclination());
          }
      }

    if(imu_->getUpdate())
      {
        /* attitude estimation */
        estimator_->update(imu_->getGyro(), imu_->getAcc(), imu_->getMag());

        /* send message to ros*/
        publish();

        /* reset update status of imu*/
        imu_->setUpdate(false);
      }
  }
#endif

  /* send message via ros protocol */
  void publish() {
    /* imu data (default: body/board frame */
    uint32_t now_time = HAL_GetTick();
    if (now_time - last_imu_pub_time_ >= IMU_PUB_INTERVAL) {
      last_imu_pub_time_ = now_time;
#ifdef SIMULATION
      imu_msg_.stamp = node_->get_clock()->now();
#else
      {
        uint64_t t_ms = rmw_uros_epoch_millis();
        imu_msg_.stamp.sec = (int32_t)(t_ms / 1000ULL);
        imu_msg_.stamp.nanosec = (uint32_t)((t_ms % 1000ULL) * 1000000ULL);
      }
#endif
      // sensor values
      for (int i = 0; i < 3; i++) {
        imu_msg_.mag[i] = estimator_->getMag()[i];
        imu_msg_.acc[i] = estimator_->getAcc()[i];
        imu_msg_.gyro[i] = estimator_->getAngular()[i];
      }

      // quaternion
      ap::Quaternion q = estimator_->getQuaternion();
      imu_msg_.quaternion[0] = q[1];  // x
      imu_msg_.quaternion[1] = q[2];  // y
      imu_msg_.quaternion[2] = q[3];  // z
      imu_msg_.quaternion[3] = q[0];  // w

#ifdef SIMULATION
      imu_pub_->publish(imu_msg_);
#else
      rcl_publish(&imu_pub_, &imu_msg_, NULL);
#endif
    }
  }

  EstimatorAlgorithm* getEstimator() { return estimator_; }

  const ap::Matrix3f getRotation() {
    if (!use_ground_truth_)
      return estimator_->getRotation();
    else
      return ground_truth_rot_;
  }

  const ap::Vector3f getAngular() {
    if (!use_ground_truth_)
      return estimator_->getAngular();
    else
      return ground_truth_ang_vel_;
  }

  inline void useGroundTruth(bool flag) { use_ground_truth_ = flag; }  // for simulation
  void setGroundTruthStates(ap::Matrix3f rot, ap::Vector3f ang_vel) {
    ground_truth_rot_ = rot;
    ground_truth_ang_vel_ = ang_vel;
  }

  void magDeclinationCallback(const spinal_msgs__srv__MagDeclination_Request& req, spinal_msgs__srv__MagDeclination_Response& res)
  {
    switch (req.command)
      {
      case spinal_msgs__srv__MagDeclination_Request__GET_DECLINATION:
        {
          res.data = estimator_->getMagDeclination();
          res.success = true;
          break;
        }
      case spinal_msgs__srv__MagDeclination_Request__SET_DECLINATION:
        {
          /* update the magnetic declination */
          estimator_->setMagDeclination(req.data);
          res.success = true;
          break;
        }
      default:
        {
          break;
        }
      }
  }

  static const uint8_t IMU_PUB_INTERVAL = 5;  // 10-> 100Hz, 2 -> 500Hz

 private:
#ifdef SIMULATION
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;  // ROS2 node handle
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::Imu>> imu_pub_;
#else
  rcl_node_t* node_;
  rclc_executor_t* executor_;
  rcl_publisher_t imu_pub_;
#endif
  spinal_msgs__msg__Imu imu_msg_;

  EstimatorAlgorithm* estimator_;
  
#ifndef SIMULATION
  IMU* imu_;
  GPS* gps_;

  rcl_service_t mag_declination_srv_;
  spinal_msgs__srv__MagDeclination_Request mag_declination_req_;
  spinal_msgs__srv__MagDeclination_Response mag_declination_res_;

#else
  ap::Vector3f acc_, mag_, gyro_;
  uint32_t HAL_GetTick() { return (uint32_t)(node_->get_clock()->now().seconds() * 1000); }
#endif

  uint32_t last_imu_pub_time_, last_attitude_pub_time_;

  bool use_ground_truth_;  // for simulation
  ap::Matrix3f ground_truth_rot_;
  ap::Vector3f ground_truth_ang_vel_;
};

#ifndef SIMULATION
namespace
{
  AttitudeEstimate* attitude_instance_ = nullptr;

  void magDeclinationCallbackStatic(const void * req_msg, void * res_msg)
  {
    if(attitude_instance_ == nullptr) return;
    if(req_msg == nullptr) return;
    if(res_msg == nullptr) return;
    attitude_instance_->magDeclinationCallback(
      *reinterpret_cast<const spinal_msgs__srv__MagDeclination_Request*>(req_msg),
      *reinterpret_cast<spinal_msgs__srv__MagDeclination_Response*>(res_msg));
  }
}
#endif

#endif
