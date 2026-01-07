/*
******************************************************************************
* File Name          : imu_ros_cmd.cpp
* Description        : IMU ROS command interface
******************************************************************************
*/


#include "imu_ros_cmd.h"
#include <vector>
#include <string>
#include "flashmemory/flashmemory.h"

namespace IMU_ROS_CMD
{
  namespace
  {
    rcl_node_t* node_;
    rclc_executor_t* executor_;
    std::vector<IMU*> imu_;
    bool first_call_ = true;
    constexpr uint8_t CALIB_DATA_SIZE = 12; // gyro_bias (3) + acc_bias (3) + mag_bias (3) + mag_scale (3)

    rcl_service_t imu_calib_srv_;
    spinal_msgs__srv__ImuCalib_Request req_;
    spinal_msgs__srv__ImuCalib_Response res_;

    static constexpr uint8_t MAX_IMU_COUNT = 4;
    static constexpr uint16_t MAX_CALIB_FLOATS = MAX_IMU_COUNT * CALIB_DATA_SIZE;
    float calib_data_buf_[MAX_CALIB_FLOATS];
  }

  void getImuCalibData()
  {
    for(unsigned int i = 0; i < imu_.size(); i++)
      {
        Vector3f gyro_bias = imu_.at(i)->getGyroBias();
        Vector3f acc_bias = imu_.at(i)->getAccBias();
        Vector3f mag_bias = imu_.at(i)->getMagBias();
        Vector3f mag_scale = imu_.at(i)->getMagScale();
        res_.data.data[i * CALIB_DATA_SIZE] = gyro_bias.x;
        res_.data.data[i * CALIB_DATA_SIZE + 1] = gyro_bias.y;
        res_.data.data[i * CALIB_DATA_SIZE + 2] = gyro_bias.z;
        res_.data.data[i * CALIB_DATA_SIZE + 3] = acc_bias.x;
        res_.data.data[i * CALIB_DATA_SIZE + 4] = acc_bias.y;
        res_.data.data[i * CALIB_DATA_SIZE + 5] = acc_bias.z;
        res_.data.data[i * CALIB_DATA_SIZE + 6] = mag_bias.x;
        res_.data.data[i * CALIB_DATA_SIZE + 7] = mag_bias.y;
        res_.data.data[i * CALIB_DATA_SIZE + 8] = mag_bias.z;
        res_.data.data[i * CALIB_DATA_SIZE + 9] = mag_scale.x;
        res_.data.data[i * CALIB_DATA_SIZE + 10] = mag_scale.y;
        res_.data.data[i * CALIB_DATA_SIZE + 11] = mag_scale.z;
      }
  }

  static void imuCalibCallbackStatic(const void * req_msg, void * res_msg)
  {
    (void)res_msg;

    if(req_msg == nullptr) return;

    if(first_call_)
      {
        /* init response buffer for imu calib */
        const uint16_t needed = imu_.size() * CALIB_DATA_SIZE;
        if(needed > MAX_CALIB_FLOATS)
          {
            first_call_ = false;
            res_.success = false;
            res_.data.size = 0;
            return;
          }

        res_.data.data = calib_data_buf_;
        res_.data.size = needed;
        res_.data.capacity = MAX_CALIB_FLOATS;

        first_call_ = false;
      }

    const spinal_msgs__srv__ImuCalib_Request* req = reinterpret_cast<const spinal_msgs__srv__ImuCalib_Request*>(req_msg);

    switch (req->command)
      {
      case spinal_msgs__srv__ImuCalib_Request__GET_CALIB_DATA:
        {
          res_.success = true;
          getImuCalibData();
          break;
        }
      case spinal_msgs__srv__ImuCalib_Request__RESET_CALIB_DATA:
        {
          for(unsigned int i = 0; i < imu_.size(); i++)
            imu_.at(i)->resetCalib();

          getImuCalibData();
          res_.success = true;

          break;
        }
      case spinal_msgs__srv__ImuCalib_Request__CALIB_GYRO:
        {
          bool calib_flag = req->data.data[0];

          for(unsigned int i = 0; i < imu_.size(); i++)
            imu_.at(i)->gyroCalib(calib_flag, req->data.data[1]);

          res_.success =true;
          res_.data.size = 0;

          break;
        }
      case spinal_msgs__srv__ImuCalib_Request__CALIB_ACC:
        {
          bool calib_flag = req->data.data[0];

          for(unsigned int i = 0; i < imu_.size(); i++)
            imu_.at(i)->accCalib(calib_flag, req->data.data[1]);

          res_.success =true;
          res_.data.size = 0;

          break;
        }
      case spinal_msgs__srv__ImuCalib_Request__CALIB_MAG:
        {
          bool calib_flag = req->data.data[0];

          for(unsigned int i = 0; i < imu_.size(); i++)
            imu_.at(i)->magCalib(calib_flag, req->data.data[1]);

          res_.success =true;
          res_.data.size = 0;
          break;
        }
      case spinal_msgs__srv__ImuCalib_Request__SEND_CALIB_DATA:
        {
          int imu_id = req->data.data[0];
          int module_id = req->data.data[1];

          res_.success = true;
          switch (module_id)
            {
            case spinal_msgs__srv__ImuCalib_Request__CALIB_GYRO:
              {
                imu_.at(imu_id)->setGyroBias(Vector3f(req->data.data[2], req->data.data[3], req->data.data[4]));
                break;
              }
            case spinal_msgs__srv__ImuCalib_Request__CALIB_ACC:
              {
                imu_.at(imu_id)->setAccBias(Vector3f(req->data.data[2], req->data.data[3], req->data.data[4]));
                break;
              }
            case spinal_msgs__srv__ImuCalib_Request__CALIB_MAG:
              {
                imu_.at(imu_id)->setMagBias(Vector3f(req->data.data[2], req->data.data[3], req->data.data[4]));
                imu_.at(imu_id)->setMagScale(Vector3f(req->data.data[5], req->data.data[6], req->data.data[7]));
                break;
              }
            default:
              {
                res_.success = false;
                break;
              }
            }

          res_.data.size = 0;
          break;
        }
      case spinal_msgs__srv__ImuCalib_Request__SAVE_CALIB_DATA:
        {
          /* because we have more than one imu, so calling IMU::writeCalibData() is very inefficient */
          FlashMemory::erase();
          FlashMemory::write();

          res_.success =true;
          res_.data.size = 0;
          break;
        }
      default:
        {
          res_.success =false;
          res_.data.size = 0;
          break;
        }
      }
  }

  void init(rcl_node_t* node, rclc_executor_t* executor)
  {
    node_ = node;
    executor_ = executor;

    spinal_msgs__srv__ImuCalib_Request__init(&req_);
    spinal_msgs__srv__ImuCalib_Response__init(&res_);

    rclc_service_init_default(
      &imu_calib_srv_,
      node_,
      ROSIDL_GET_SRV_TYPE_SUPPORT(spinal_msgs, srv, ImuCalib),
      "imu_calib");

    rclc_executor_add_service(
      executor_,
      &imu_calib_srv_,
      &req_,
      &res_,
      imuCalibCallbackStatic);
  }

  void addImu(IMU* imu)
  {
    imu_.push_back(imu);
  }
}
