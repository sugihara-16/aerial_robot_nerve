#include "sensors/imu/imu_ros_module.h"

ImuRosModule* ImuRosModule::instance_ = nullptr;

void ImuRosModule::addImu(IMU* imu)
{
  if (!imu) return;
  imu_.push_back(imu);
}

void ImuRosModule::getImuCalibData_()
{
  for (unsigned int i = 0; i < imu_.size(); i++)
  {
    Vector3f gyro_bias  = imu_.at(i)->getGyroBias();
    Vector3f acc_bias   = imu_.at(i)->getAccBias();
    Vector3f mag_bias   = imu_.at(i)->getMagBias();
    Vector3f mag_scale  = imu_.at(i)->getMagScale();

    res_.data.data[i * CALIB_DATA_SIZE + 0]  = gyro_bias.x;
    res_.data.data[i * CALIB_DATA_SIZE + 1]  = gyro_bias.y;
    res_.data.data[i * CALIB_DATA_SIZE + 2]  = gyro_bias.z;
    res_.data.data[i * CALIB_DATA_SIZE + 3]  = acc_bias.x;
    res_.data.data[i * CALIB_DATA_SIZE + 4]  = acc_bias.y;
    res_.data.data[i * CALIB_DATA_SIZE + 5]  = acc_bias.z;
    res_.data.data[i * CALIB_DATA_SIZE + 6]  = mag_bias.x;
    res_.data.data[i * CALIB_DATA_SIZE + 7]  = mag_bias.y;
    res_.data.data[i * CALIB_DATA_SIZE + 8]  = mag_bias.z;
    res_.data.data[i * CALIB_DATA_SIZE + 9]  = mag_scale.x;
    res_.data.data[i * CALIB_DATA_SIZE + 10] = mag_scale.y;
    res_.data.data[i * CALIB_DATA_SIZE + 11] = mag_scale.z;
  }
}

void ImuRosModule::imuCalibCallbackStatic_(const void* req_msg, void* res_msg)
{
  (void)res_msg;
  if (!instance_) return;
  if (req_msg == nullptr) return;

  if (instance_->first_call_)
  {
    const uint16_t needed = instance_->imu_.size() * CALIB_DATA_SIZE;
    if (needed > MAX_CALIB_FLOATS)
    {
      instance_->first_call_ = false;
      instance_->res_.success = false;
      instance_->res_.data.size = 0;
      return;
    }

    instance_->res_.data.data = instance_->calib_data_buf_;
    instance_->res_.data.size = needed;
    instance_->res_.data.capacity = MAX_CALIB_FLOATS;

    instance_->first_call_ = false;
  }

  const auto* req = reinterpret_cast<const spinal_msgs__srv__ImuCalib_Request*>(req_msg);

  switch (req->command)
  {
    case spinal_msgs__srv__ImuCalib_Request__GET_CALIB_DATA:
    {
      instance_->res_.success = true;
      instance_->getImuCalibData_();
      break;
    }

    case spinal_msgs__srv__ImuCalib_Request__RESET_CALIB_DATA:
    {
      for (unsigned int i = 0; i < instance_->imu_.size(); i++)
        instance_->imu_.at(i)->resetCalib();

      instance_->getImuCalibData_();
      instance_->res_.success = true;
      break;
    }

    case spinal_msgs__srv__ImuCalib_Request__CALIB_GYRO:
    {
      bool calib_flag = req->data.data[0];
      for (unsigned int i = 0; i < instance_->imu_.size(); i++)
        instance_->imu_.at(i)->gyroCalib(calib_flag, req->data.data[1]);

      instance_->res_.success = true;
      instance_->res_.data.size = 0;
      break;
    }

    case spinal_msgs__srv__ImuCalib_Request__CALIB_ACC:
    {
      bool calib_flag = req->data.data[0];
      for (unsigned int i = 0; i < instance_->imu_.size(); i++)
        instance_->imu_.at(i)->accCalib(calib_flag, req->data.data[1]);

      instance_->res_.success = true;
      instance_->res_.data.size = 0;
      break;
    }

    case spinal_msgs__srv__ImuCalib_Request__CALIB_MAG:
    {
      bool calib_flag = req->data.data[0];
      for (unsigned int i = 0; i < instance_->imu_.size(); i++)
        instance_->imu_.at(i)->magCalib(calib_flag, req->data.data[1]);

      instance_->res_.success = true;
      instance_->res_.data.size = 0;
      break;
    }

    case spinal_msgs__srv__ImuCalib_Request__SEND_CALIB_DATA:
    {
      int imu_id    = req->data.data[0];
      int module_id = req->data.data[1];

      instance_->res_.success = true;

      switch (module_id)
      {
        case spinal_msgs__srv__ImuCalib_Request__CALIB_GYRO:
          instance_->imu_.at(imu_id)->setGyroBias(Vector3f(req->data.data[2], req->data.data[3], req->data.data[4]));
          break;

        case spinal_msgs__srv__ImuCalib_Request__CALIB_ACC:
          instance_->imu_.at(imu_id)->setAccBias(Vector3f(req->data.data[2], req->data.data[3], req->data.data[4]));
          break;

        case spinal_msgs__srv__ImuCalib_Request__CALIB_MAG:
          instance_->imu_.at(imu_id)->setMagBias(Vector3f(req->data.data[2], req->data.data[3], req->data.data[4]));
          instance_->imu_.at(imu_id)->setMagScale(Vector3f(req->data.data[5], req->data.data[6], req->data.data[7]));
          break;

        default:
          instance_->res_.success = false;
          break;
      }

      instance_->res_.data.size = 0;
      break;
    }

    case spinal_msgs__srv__ImuCalib_Request__SAVE_CALIB_DATA:
    {

      FlashMemory::erase();
      FlashMemory::write();

      instance_->res_.success = true;
      instance_->res_.data.size = 0;
      break;
    }

    default:
      instance_->res_.success = false;
      instance_->res_.data.size = 0;
      break;
  }
}

void ImuRosModule::create_entities(rcl_node_t& node)
{
  reserve_entities();
  instance_ = this;

  spinal_msgs__srv__ImuCalib_Request__init(&req_);
  spinal_msgs__srv__ImuCalib_Response__init(&res_);


  (void)init_service_default(
      node,
      imu_calib_srv_,
      ROSIDL_GET_SRV_TYPE_SUPPORT(spinal_msgs, srv, ImuCalib),
      "imu_calib",
      &req_,
      &res_,
      &ImuRosModule::imuCalibCallbackStatic_);
}
