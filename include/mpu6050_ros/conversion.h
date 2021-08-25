/**
 * @author  Dwindra Sulistyoutomo
 */

#pragma once

#include <sensor_msgs/Imu.h>

#ifndef G_FORCE
#define G_FORCE 9.80665
#endif

namespace mpu6050_conversion
{
const float degToRad = M_PI/180.0;

/**
 * Create IMU msg from DMP FIFO buffer
 * 
 * @param fifo_buffer {uint8_t*}  DMP FIFO Buffer
 * @param accel_scale {float}     accelerometer scale/sensitivity
 * @param gyro_scale {float}      gyro scale/sensitivity
 * @param frame_id {string}       frame id for the message
 * 
 * @return {sensor_msgs::Imu}
 */
sensor_msgs::Imu GenerateImuMsg (uint8_t* fifo_buffer, float accel_scale, float gyro_scale, std::string frame_id="/frame");

}