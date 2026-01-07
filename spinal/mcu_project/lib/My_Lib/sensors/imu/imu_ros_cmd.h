/*
******************************************************************************
* File Name          : imu_ros_cmd.h
* Description        : IMU ROS command interface
******************************************************************************
*/


#ifndef APPLICATION_JSK_LIB_SENSORS_IMU_IMU_ROS_CMD_H_
#define APPLICATION_JSK_LIB_SENSORS_IMU_IMU_ROS_CMD_H_

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <spinal_msgs/srv/imu_calib.h>
#include "sensors/imu/imu_basic.h"

namespace IMU_ROS_CMD {
	void init(rcl_node_t* node, rclc_executor_t* executor);
	void addImu(IMU* imu);
};



#endif /* APPLICATION_JSK_LIB_SENSORS_IMU_IMU_ROS_CMD_H_ */
