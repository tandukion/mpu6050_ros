/**
 * @author  Dwindra Sulistyoutomo
 */

#ifndef _MPU6050_HANDLER_H
#define _MPU6050_HANDLER_H

#include "ros/ros.h"
#include "MPU6050Pi.h"
#include "mpu6050_ros/conversion.h"

#define COMPLEMENTARY_FILTER_CONSTANT   0.98
#define DEFAULT_RATE                    100

class MPU6050Handler {
  private:
    // Main class members
    ros::NodeHandle nh_;
    MPU6050Pi mpu_;
    bool dmp_;
    uint16_t packet_size_;

    // Data update thread
    ros::Rate update_rate_;
    std::thread update_task_;
    std::thread update_dmp_task_;

  public:
    int dev_status;
    float accel_scale;
    float gyro_scale;
    uint8_t fifo_buffer[64];

    float gyro_deg[3];  // gyro data in deg/s
    float accel_g[3];   // accel data in g
    float gyro[3];      // gyro data in rad/s
    float accel[3];     // accel data in m/s^2
    float rpy[3];       // roll pitch yaw
    float rpy_comp[3];  // complementary roll pitch yaw

    /**
     * Constructor
     * 
     * @param nh {ros::NodeHandle}  ROS Node Handler
     * @param offsets{int16_t*}     List of MPU6050 offsets
     * @param rate {int16_t}        frequency of data update
     * @param dmp {bool}            flag to use DMP or not
     */
    MPU6050Handler(ros::NodeHandle *nh, int16_t *offsets, int16_t rate=DEFAULT_RATE, bool dmp=false);

    /**
     * Update MPU6050 Data
     * Process the gyroscope and accelerometer to 
     */
    void UpdateData(int16_t rate);

    /**
     * Update DMP FIFO buffer
     */
    void UpdateDMP();

    /**
     * Return IMU msg data ready to be published
     * 
     * @return {sensor_msgs::Imu} IMU msg data
     */
    sensor_msgs::Imu GetImuMsg();

    /**
     * Calculate angle using Complementary Filter
     * Complementary Filter is implemented with first order High-Pass and Low-Pass filter.  
     */
    float ComplementaryFilter(float angle, float angle_comp) {
      return COMPLEMENTARY_FILTER_CONSTANT * angle + (1- COMPLEMENTARY_FILTER_CONSTANT) * angle_comp;
    }

};

#endif