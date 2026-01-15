/*
******************************************************************************
* File Name          : state_estimate.h
* Description        : state(attitude, altitude, pos) estimate core (NO ROS)
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __STATE_ESTIMATE_H
#define __STATE_ESTIMATE_H

#ifndef SIMULATION
#include "config.h"
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"
#include "sensors/imu/drivers/icm20948/icm_20948.h"
#include "sensors/baro/baro_ms5611.h"
#include "sensors/gps/gps_ublox.h"
#else

#endif

#include "state_estimate/attitude/attitude_estimate.h"

class StateEstimate {
 public:
  StateEstimate() = default;
  ~StateEstimate() = default;

#ifdef SIMULATION
  void init()
  {
    attitude_estimate_flag_ = true;
    attitude_estimator_.init();
    last_imu_update_time_ms_ = nowMsStub_();
  }
#else
  void init(IMU* imu, Baro* baro, GPS* gps)
  {
    imu_  = imu;
    baro_ = baro;
    gps_  = gps;

    if (imu_ == nullptr) {
      attitude_estimate_flag_ = false;
    } else {
      attitude_estimate_flag_ = true;
      attitude_estimator_.init(imu_, gps_);
    }

    altitude_estimate_flag_ = (baro_ != nullptr);
    pos_estimate_flag_      = (gps_  != nullptr);

    last_imu_update_time_ms_ = HAL_GetTick();
  }
#endif  // SIMULATION

  void update()
  {
    if (attitude_estimate_flag_) {
      attitude_estimator_.update();
      last_imu_update_time_ms_ = nowMs_();
    }

#ifndef SIMULATION
    // if (altitude_estimate_flag_) altitude_estimator_.update();
    // if (pos_estimate_flag_) pos_estimator_.update();
#endif
  }

  AttitudeEstimate* getAttEstimator() { return &attitude_estimator_; }

#ifndef SIMULATION
  IMU*  getImu()  { return imu_;  }
  Baro* getBaro() { return baro_; }
  GPS*  getGPS()  { return gps_;  }
#endif

  bool attitudeEnabled() const { return attitude_estimate_flag_; }
  bool altitudeEnabled() const { return altitude_estimate_flag_; }
  bool posEnabled() const { return pos_estimate_flag_; }

 private:
#ifdef SIMULATION
  uint32_t nowMsStub_() const { return 0; }
#endif

  uint32_t nowMs_() const
  {
#ifdef SIMULATION
    return nowMsStub_();
#else
    return HAL_GetTick();
#endif
  }

 private:
#ifndef SIMULATION
  IMU*  imu_  = nullptr;
  Baro* baro_ = nullptr;
  GPS*  gps_  = nullptr;
#endif

  uint32_t last_imu_update_time_ms_{0};

  AttitudeEstimate attitude_estimator_;

  bool attitude_estimate_flag_{false};
  bool altitude_estimate_flag_{false};
  bool pos_estimate_flag_{false};
};

#endif  // __STATE_ESTIMATE_H
