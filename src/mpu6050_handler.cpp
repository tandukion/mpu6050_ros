/**
 * @author  Dwindra Sulistyoutomo
 */

#include "mpu6050_ros/mpu6050_handler.h"

MPU6050Handler::MPU6050Handler(ros::NodeHandle *nh, int16_t *offsets, int16_t rate, bool dmp)
  : // initialization list
  nh_(*nh),
  mpu_(MPU6050Pi()),
  dmp_(dmp),
  update_rate_(ros::Rate(rate))
{
  // Set MPU6050 Offsets
  mpu_.SetOffset(offsets);

  accel_scale = mpu_.GetAccelSensitivity();
  gyro_scale = mpu_.GetGyroSensitivity();

  if (!dmp) {
    ROS_INFO("MPU6050 data update thread started");
    update_task_ = std::thread(&MPU6050Handler::UpdateData, this, rate);
  }
  else {
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
}

void MPU6050Handler::UpdateData(int16_t rate) {
  uint8_t i;
  // Initialize angle data
  for (i=0;i<3;i++) {
    rpy[i] = 0;
    rpy_comp[i] = 0;
  }
  while (ros::ok()) {
    // Get IMU data
    mpu_.GetGyroFloat(&gyro_deg[0], &gyro_deg[1], &gyro_deg[2]);
    mpu_.GetAccelFloat(&accel_g[0], &accel_g[1], &accel_g[2]);

    // Converting data. Gyro needs to be in rad/s, accel needs to be in m/s^2
    for (i=0;i<3;i++) {
      gyro[i] = gyro_deg[i] * M_PI/180.0;
      accel[i] = accel_g[i] * G_FORCE;
    }

    // Calculate the angles in radians.
    // 1. Roll
    //    From gyro. gyro data is in deg/s.
    rpy[0] += gyro[0] * 1.0/rate;
    //    From accel. atan or atan2f return is in rad
    rpy_comp[0] = atan2f(accel[1], sqrt(accel[0]*accel[0] + accel[2]*accel[2]));
    // Complementary Filter
    rpy[0] = MPU6050Handler::ComplementaryFilter(rpy[0], rpy_comp[0]);

    // 2. Pitch
    //    From gyro. gyro data is in deg/s.
    rpy[1] += gyro[1] * 1.0/rate;
    //    From accel. atan or atan2f return is in rad
    rpy_comp[1] = atan2f(accel[0], sqrt(accel[1]*accel[1] + accel[2]*accel[2]));
    // Complementary Filter
    rpy[1] = MPU6050Handler::ComplementaryFilter(rpy[1], rpy_comp[1]);

    // 3. Yaw
    //    From gyro. gyro data is in deg/s.
    rpy[2] += gyro[2] * 1.0/rate;
    
    // Maintain refresh rate
    update_rate_.sleep();
  }
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

sensor_msgs::Imu MPU6050Handler::GetImuMsg() {
  sensor_msgs::Imu msg;
  if (!dmp_) {
    msg = mpu6050_conversion::GenerateImuMsg(rpy, gyro, accel);
  }
  else {
    msg = mpu6050_conversion::GenerateImuMsg(fifo_buffer, accel_scale, gyro_scale);
  }
  return msg;
}