/**
 * @author  Dwindra Sulistyoutomo
 */

#ifndef _MPU6050_HANDLER_H
#define _MPU6050_HANDLER_H

#include "ros/ros.h"
#include "MPU6050Pi.h"

#define COMPLEMENTARY_FILTER_CONSTANT   0.98
#define DEFAULT_RATE                    100

class MPU6050Handler {
  private:
    // Main class members
    ros::NodeHandle nh_;
    MPU6050Pi mpu_;
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

    float gyro[3];
    float accel[3];
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

    void UpdateData(int16_t rate);

    /**
     * Update DMP FIFO buffer
     */
    void UpdateDMP();

    float ComplementaryFilter(float angle, float angle_comp) {
      return COMPLEMENTARY_FILTER_CONSTANT * angle + (1- COMPLEMENTARY_FILTER_CONSTANT) * angle_comp;
    }

};

#endif