/**
 * @author  Dwindra Sulistyoutomo
 */

#include "mpu6050_ros/conversion.h"

namespace mpu6050_conversion
{

sensor_msgs::Imu CreateImuMsg (uint8_t* fifo_buffer, float accel_scale, float gyro_scale, frame_id='') {
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;

  // Quaternion
  msg.orientation.w = (float)((fifo_buffer[0] << 8) | fifo_buffer[1]) / accel_scale;
  msg.orientation.x = (float)((fifo_buffer[4] << 8) | fifo_buffer[5]) / accel_scale;
  msg.orientation.y = (float)((fifo_buffer[8] << 8) | fifo_buffer[9]) / accel_scale;
  msg.orientation.z = (float)((fifo_buffer[12] << 8) | fifo_buffer[13]) / accel_scale;

  // Gyro
  msg.angular_velocity.x = (float)((fifo_buffer[16] << 8) | fifo_buffer[17]) / gyro_scale;
  msg.angular_velocity.y = (float)((fifo_buffer[20] << 8) | fifo_buffer[21]) / gyro_scale;
  msg.angular_velocity.z = (float)((fifo_buffer[24] << 8) | fifo_buffer[25]) / gyro_scale;

  // Accel
  msg.linear_acceleration.x = (float)((fifo_buffer[28] << 8) | fifo_buffer[29]) * G_FORCE/accel_scale;
  msg.linear_acceleration.y = (float)((fifo_buffer[32] << 8) | fifo_buffer[33]) * G_FORCE/accel_scale;
  msg.linear_acceleration.z = (float)((fifo_buffer[36] << 8) | fifo_buffer[37]) * G_FORCE/accel_scale;

  return msg;
}
  
}