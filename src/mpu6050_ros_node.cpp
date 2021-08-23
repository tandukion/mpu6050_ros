/**
 * @author  Dwindra Sulistyoutomo
 */

#include "ros/ros.h"
#include "MPU6050Pi.h"

#include "mpu6050_ros/conversion.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpu6050_node");
  ROS_INFO("MPU6050 node created");

  // Connect to device with default setting
  MPU6050Pi mpu;

  // MPU6050 variables
  float accel_scale = mpu.GetAccelSensitivity();
  float gyro_scale = mpu.GetGyroSensitivity();
  int dev_status;
  uint16_t packet_size;
  uint16_t fifo_count;
  uint8_t fifo_buffer[64];

  // MPU6050 data
  Quaternion q;

  ros::NodeHandle nh;

  ros::Publisher raw_data_pub = nh.advertise<sensor_msgs::Imu>("raw_data", 10);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

  ROS_INFO("Initializing DMP..");
  dev_status = mpu.DMPInitalize();

  // make sure it worked (returns 0 if so)
  if (dev_status == 0) {
      mpu.SetDMPEnabled(true);
      ROS_INFO("DMP enabled");
      packet_size = mpu.DMPGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      ROS_ERROR("DMP Initialization failed.\n Error code: %s",dev_status);
      return 0;
  }
  
  ros::Rate loop_rate(10);

  // Publish in loop.
  std::cout << "Quaternion\n";
  std::cout << std::fixed << std::setprecision(6) << std::setfill(' ');
  std::cout << std::setw(12) << "X" << std::setw(12) << "Y" << std::setw(12) << "Z" << std::setw(12) << "W";
  std::cout << std::endl;

  while (ros::ok()) {
    // Clear the buffer so as we can get fresh values
    // The sensor is running a lot faster than our sample period
    mpu.ResetFIFO();
    
    // Waiting until FIFO full on MPU6050 side
    while (!mpu.DMPPacketAvailable()) {}

    // Read FIFO buffer
    mpu.GetFIFOBytes(fifo_buffer,packet_size);

    // Process FIFO buffer to get quaternion
    mpu.DMPGetQuaternion(&q, fifo_buffer);
    std::cout << std::setw(12) << q.x << std::setw(12) << q.y << std::setw(12) << q.z << std::setw(12) << q.w;
    std::cout << "\r";

    // Create IMU msg
    sensor_msgs::Imu msg = mpu6050_conversion::CreateImuMsg(fifo_buffer, accel_scale, gyro_scale);

    // Publish
    imu_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}