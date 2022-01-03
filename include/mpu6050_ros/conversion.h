/**
 * @author  Dwindra Sulistyoutomo
 */

#pragma once

#include <fstream>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "math_3d.h"

#ifndef G_FORCE
#define G_FORCE 9.80665
#endif

namespace mpu6050_conversion
{
const float degToRad = M_PI/180.0;

/**
 * Return the MPU6050 calibration data from the given filepath. File should be in CSV.
 * 
 * @param filepath {string} path for the calibration data file
 * 
 * @return {int16_t*}   Array of calibration data
 */
int16_t* GetCalibrationData (std::string filepath);

/**
 * Create IMU msg from XYZ rotation angle, gyroscope and accelerometer data
 * 
 * @param rpy {float*}      rotation angle in XYZ axis (roll, pitch,yaw)
 * @param gyro {float*}     gyroscope data in rad/s
 * @param accel {float*}    accelerometer data in m/s^2
 * @param frame_id {string} frame id for the message
 * 
 * @return {sensor_msgs::Imu}
 */
sensor_msgs::Imu GenerateImuMsg (float* rpy, float* gyro, float* accel, std::string frame_id="imu");

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
sensor_msgs::Imu GenerateImuMsg (uint8_t* fifo_buffer, float gyro_scale, float accel_scale, int accel_scale_range=2, std::string frame_id="imu");

/**
 * Create IMU msg from DMP FIFO buffer
 * 
 * @param q {Quaternion}          Quaternion data
 * @param v_gyro {Vector}         gyroscope data (in deg/s)
 * @param v_accel {Vector}        accelerometer data (in g)
 * @param frame_id {string}       frame id for the message
 * 
 * @return {sensor_msgs::Imu}
 */
sensor_msgs::Imu GenerateImuMsg (Quaternion q, Vector v_gyro, Vector v_accel, std::string frame_id="imu");

}