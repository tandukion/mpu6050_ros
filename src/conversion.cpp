/**
 * @author  Dwindra Sulistyoutomo
 */

#include "mpu6050_ros/conversion.h"

namespace mpu6050_conversion
{

int16_t* GetCalibrationData (std::string filepath) {
  int16_t *offsets = new int16_t[6];
  std::fill(offsets, offsets+6, 0);
  
  // Getting the file extension
  std::string filename = filepath.substr(filepath.find_last_of("/\\")+1);
  if (filename.find_last_of(".") != std::string::npos) {
    // Get file extension
    std::string file_ext = filename.substr(filename.find_last_of("."));
    
    // Make sure the calibration file is CSV
    if ((file_ext == ".csv") || (file_ext == ".CSV")) {
      std::ifstream ifile (filepath);
      std::string header;
      std::string data_str;
      if (ifile.is_open()){
        // Getting data line by line
        // 1st line should be Header
        std::getline(ifile,header);

        // Data
        std::getline(ifile,data_str);

        ifile.close();

        // Parse csv
        int i=0;
        size_t pos = 0;
        std::string token;
        std::string delimiter = ",";
        while ((pos = data_str.find(delimiter)) != std::string::npos) {
          offsets[i] = stoi(data_str.substr(0, pos));
          data_str.erase(0, pos + delimiter.length());
          i++;
        }
        // Handle last data if not ending with comma
        if (data_str.length() > 0){
          offsets[i] = stoi(data_str);
        }
      }
      return offsets;
    }
    else {
      std::cout << "Wrong input file. Calibration file should be CSV file." << std::endl;
      return offsets;
    }
  }
  else {
    std::cout << "Wrong input file. Calibration file should be CSV file." << std::endl;
    return offsets;
  }
}

sensor_msgs::Imu GenerateImuMsg (float* rpy, float* gyro, float* accel, std::string frame_id) {
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;

  // Quaternion
  // convert the rpy to quaternion
  tf2::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
  tf2::convert(q, msg.orientation);

  // Gyro
  // Need to be converted to rad/s
  msg.angular_velocity.x = gyro[0];
  msg.angular_velocity.y = gyro[1];
  msg.angular_velocity.z = gyro[2];

  // Accel
  // Need to be in m/s
  msg.linear_acceleration.x = accel[0];
  msg.linear_acceleration.y = accel[1];
  msg.linear_acceleration.z = accel[2];

  return msg;
}

sensor_msgs::Imu GenerateImuMsg (uint8_t* fifo_buffer, float gyro_scale, float accel_scale, int accel_scale_range, std::string frame_id) {
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;

  // Quaternion
  msg.orientation.w = (float)(int16_t)((fifo_buffer[0] << 8)  | fifo_buffer[1])  / accel_scale;
  msg.orientation.x = (float)(int16_t)((fifo_buffer[4] << 8)  | fifo_buffer[5])  / accel_scale;
  msg.orientation.y = (float)(int16_t)((fifo_buffer[8] << 8)  | fifo_buffer[9])  / accel_scale;
  msg.orientation.z = (float)(int16_t)((fifo_buffer[12] << 8) | fifo_buffer[13]) / accel_scale;

  // Gyro
  // Need to be converted to rad/s
  msg.angular_velocity.x = (float)(int16_t)((fifo_buffer[16] << 8) | fifo_buffer[17]) / gyro_scale * degToRad;
  msg.angular_velocity.y = (float)(int16_t)((fifo_buffer[20] << 8) | fifo_buffer[21]) / gyro_scale * degToRad;
  msg.angular_velocity.z = (float)(int16_t)((fifo_buffer[24] << 8) | fifo_buffer[25]) / gyro_scale * degToRad;

  // Accel
  // Need to be in m/s
  msg.linear_acceleration.x = (float)(int16_t)((fifo_buffer[28] << 8) | fifo_buffer[29]) / accel_scale * accel_scale_range * G_FORCE;
  msg.linear_acceleration.y = (float)(int16_t)((fifo_buffer[32] << 8) | fifo_buffer[33]) / accel_scale * accel_scale_range * G_FORCE;
  msg.linear_acceleration.z = (float)(int16_t)((fifo_buffer[36] << 8) | fifo_buffer[37]) / accel_scale * accel_scale_range * G_FORCE;

  return msg;
}

sensor_msgs::Imu GenerateImuMsg (Quaternion q, Vector v_gyro, Vector v_accel, std::string frame_id) {
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;

  // Quaternion
  msg.orientation.w = q.w;
  msg.orientation.x = q.x;
  msg.orientation.y = q.y;
  msg.orientation.z = q.z;

  // Gyro
  // Need to be converted to rad/s
  msg.angular_velocity.x = (float)(v_gyro.x) * degToRad;
  msg.angular_velocity.y = (float)(v_gyro.y) * degToRad;
  msg.angular_velocity.z = (float)(v_gyro.z) * degToRad;

  // Accel
  // Need to be in m/s^2
  msg.linear_acceleration.x = (float)(v_accel.x) * G_FORCE;
  msg.linear_acceleration.y = (float)(v_accel.y) * G_FORCE;
  msg.linear_acceleration.z = (float)(v_accel.z) * G_FORCE;

  return msg;
}
  
}