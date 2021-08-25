/**
 * @author  Dwindra Sulistyoutomo
 */

#include "ros/ros.h"

#include "mpu6050_ros/mpu6050_handler.h"
#include "mpu6050_ros/conversion.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpu6050_node");
  ROS_INFO("MPU6050 node created");

  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

  // Start the MPU6050 data handler. Background thread will update the data
  MPU6050Handler mpu_handler(&nh);
  
  while (ros::ok()) {
    // Publish IMU msg
    sensor_msgs::Imu msg = mpu6050_conversion::GenerateImuMsg(mpu_handler.fifo_buffer, mpu_handler.accel_scale, mpu_handler.gyro_scale);   
    imu_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}