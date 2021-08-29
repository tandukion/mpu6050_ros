/**
 * @author  Dwindra Sulistyoutomo
 */

#include "ros/ros.h"

#include "mpu6050_ros/mpu6050_handler.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpu6050_node");
  ROS_INFO("MPU6050 node created");

  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

  // Get MPU6050 calibration data for offsets
  std::string filepath = std::getenv("HOME");
  filepath += "/.local/share/MPU6050/calibration.csv";
  ROS_INFO("Using calibration data from: %s", filepath.c_str());

  int16_t *offsets;
  offsets = mpu6050_conversion::GetCalibrationData(filepath);

  // Start the MPU6050 data handler. Background thread will update the data
  MPU6050Handler mpu_handler(&nh, offsets);

  float m[] = {-1, 0, 0, 0, 0, -1, 0, -1, 0};
  Eigen::Matrix3f M(m);
  mpu_handler.SetRotationMatrix(M);

  // Start getting MPU6050 Data
  mpu_handler.Start();
  
  while (ros::ok()) {
    // Publish IMU msg
    sensor_msgs::Imu msg = mpu_handler.GetImuMsg();   
    imu_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}