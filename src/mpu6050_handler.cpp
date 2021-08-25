/**
 * @author  Dwindra Sulistyoutomo
 */

#include "mpu6050_ros/mpu6050_handler.h"

MPU6050Handler::MPU6050Handler(ros::NodeHandle *nh)
  : // initialization list
  nh_(*nh),
  mpu_(MPU6050Pi()),
  update_rate_(ros::Rate(100))
{
  accel_scale = mpu_.GetAccelSensitivity();
  gyro_scale = mpu_.GetGyroSensitivity();

  ROS_INFO("Initializing DMP..");
  dev_status = mpu_.DMPInitalize();

  // make sure it worked (returns 0 if so)
  if (dev_status == 0) {
      mpu_.SetDMPEnabled(true);
      ROS_INFO("DMP enabled");
      packet_size_ = mpu_.DMPGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      ROS_ERROR("DMP Initialization failed.\n Error code: %s",dev_status);
  }

  // // Start thread to update the data
  ROS_INFO("DMP update thread started");
  update_dmp_task_ = std::thread(&MPU6050Handler::UpdateDMP, this);
}

void MPU6050Handler::UpdateDMP() {
  while (ros::ok()) {
    // Clear the buffer so as we can get fresh values
    // The sensor is running a lot faster than our sample period
    mpu_.ResetFIFO();
    
    // Waiting until FIFO full on MPU6050 side
    while (!mpu_.DMPPacketAvailable()) {}

    // Read FIFO buffer
    mpu_.GetFIFOBytes(fifo_buffer,packet_size_);

    // Maintain refresh rate
    update_rate_.sleep();
  }
}