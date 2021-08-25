/**
 * @author  Dwindra Sulistyoutomo
 */

#ifndef _MPU6050_HANDLER_H
#define _MPU6050_HANDLER_H

#include "ros/ros.h"
#include "MPU6050Pi.h"

class MPU6050Handler {
  private:
    // Main class members
    ros::NodeHandle nh_;
    MPU6050Pi mpu_;
    uint16_t packet_size_;

    // Data update thread
    ros::Rate update_rate_;
    std::thread update_dmp_task_;

  public:
    int dev_status;
    float accel_scale;
    float gyro_scale;
    uint8_t fifo_buffer[64];

    /**
     * Constructor
     * 
     * @param nh {ros::NodeHandle}  ROS Node Handler
     * @param offsets{int16_t*}     List of MPU6050 offsets
     */
    MPU6050Handler(ros::NodeHandle *nh, int16_t *offsets);

    /**
     * Update DMP FIFO buffer
     */
    void UpdateDMP();

};

#endif