/**
 * @author  Dwindra Sulistyoutomo
 */

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>


void ImuDataCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = msg->header.frame_id;
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;

  // Convert to tf2 quaternion
  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);

  // Filter noise from the sensor
  float ERROR_LIM = 0.001;
  if (abs(q.length()-1) > ERROR_LIM){
    // Do not publish the noise
    return;
  }

  // Normalize the quaternion, then convert to msg
  q.normalize();
  transformStamped.transform.rotation = tf2::toMsg(q);

  // Publish
  br.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpu6050_tf_broadcaster");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("imu_data", 10, ImuDataCallback);

  ros::spin();
  return 0;
}