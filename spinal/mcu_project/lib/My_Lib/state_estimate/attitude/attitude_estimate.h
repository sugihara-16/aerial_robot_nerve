/*
******************************************************************************
* File Name          : attitude_estimate.h
* Description        : attitude estimate interface (ROS-independent)
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
/* sensors (non-simulation) */
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"
#include "sensors/imu/drivers/icm20948/icm_20948.h"
#include "sensors/gps/gps_ublox.h"
#else
#include <math/AP_Math.h>
#endif

/* estimator algorithm */
#include "state_estimate/attitude/complementary_ahrs.h"
//#include "state_estimate/attitude/madgwick_ahrs.h"

#define COMPLEMENTARY 1
#define MADWICK 2
#define ESTIMATE_TYPE COMPLEMENTARY

class AttitudeEstimate {
 public:
#ifdef SIMULATION
  AttitudeEstimate()
    : acc_(0, 0, 9.8), mag_(1, 0, 0), gyro_(0, 0, 0) {}
#else
  AttitudeEstimate() = default;
#endif

  ~AttitudeEstimate() {
    if (estimator_) {
      delete estimator_;
      estimator_ = nullptr;
    }
  }

  AttitudeEstimate(const AttitudeEstimate&) = delete;
  AttitudeEstimate& operator=(const AttitudeEstimate&) = delete;

#ifndef SIMULATION
  void init(IMU* imu, GPS* gps) {
    imu_ = imu;
    gps_ = gps;
    updated_ = false;

#if ESTIMATE_TYPE == COMPLEMENTARY
    estimator_ = new ComplementaryAHRS();
#elif ESTIMATE_TYPE == MADWICK
    estimator_ = new MadgwickAHRS();
#else
#error "no instance for estimator"
#endif
  }

  void update() {
    if (!imu_ || !estimator_) return;

    // Update magnetic declination from GPS once (same behavior as original)
    if (gps_) {
      if (gps_->getMagValid() && !estimator_->getMagDecValid()) {
        estimator_->setMagDeclination(gps_->getMagDeclination());
      }
    }

    if (imu_->getUpdate()) {
      estimator_->update(imu_->getGyro(), imu_->getAcc(), imu_->getMag());
      imu_->setUpdate(false);
      updated_ = true;
    }
  }
#else
  void init() {
    updated_ = false;

#if ESTIMATE_TYPE == COMPLEMENTARY
    estimator_ = new ComplementaryAHRS();
#elif ESTIMATE_TYPE == MADWICK
    estimator_ = new MadgwickAHRS();
#else
#error "no instance for estimator"
#endif
  }

  void update() {
    if (!estimator_) return;
    estimator_->update(gyro_, acc_, mag_);
    updated_ = true;
  }

  // simulation setters
  void setMag(float x, float y, float z) { mag_.x = x; mag_.y = y; mag_.z = z; }
  void setGyro(float x, float y, float z) { gyro_.x = x; gyro_.y = y; gyro_.z = z; }
  void setAcc(float x, float y, float z) { acc_.x = x; acc_.y = y; acc_.z = z; }
#endif

  bool consumeUpdated() {
    const bool was = updated_;
    updated_ = false;
    return was;
  }

  // --------- Outputs (used by StateEstimate for publishing / services) ----------
  const ap::Vector3f& getMagVec() const { return estimator_->getMag(); }
  const ap::Vector3f& getAccVec() const { return estimator_->getAcc(); }
  const ap::Vector3f& getGyroVec() const { return estimator_->getAngular(); }
  ap::Quaternion getQuaternion() const { return estimator_->getQuaternion(); }

#ifndef SIMULATION
  float getMagDeclination() const { return estimator_->getMagDeclination(); }
  void setMagDeclination(float v) { estimator_->setMagDeclination(v); }
#endif
  EstimatorAlgorithm* getEstimator() { return estimator_; }  // optional

  // Ground truth passthrough (ROS-independent)
  const ap::Matrix3f getRotation() {
    if (!use_ground_truth_) return estimator_->getRotation();
    return ground_truth_rot_;
  }

  const ap::Vector3f getAngular() {
    if (!use_ground_truth_) return estimator_->getAngular();
    return ground_truth_ang_vel_;
  }

  inline void useGroundTruth(bool flag) { use_ground_truth_ = flag; }
  void setGroundTruthStates(ap::Matrix3f rot, ap::Vector3f ang_vel) {
    ground_truth_rot_ = rot;
    ground_truth_ang_vel_ = ang_vel;
  }

 private:
  EstimatorAlgorithm* estimator_ = nullptr;

#ifndef SIMULATION
  IMU* imu_ = nullptr;
  GPS* gps_ = nullptr;
#else
  ap::Vector3f acc_, mag_, gyro_;
#endif

  bool updated_ = false;

  bool use_ground_truth_ = false;
  ap::Matrix3f ground_truth_rot_;
  ap::Vector3f ground_truth_ang_vel_;
};

#endif
