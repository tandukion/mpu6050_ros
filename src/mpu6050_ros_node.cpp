/**
 * @author  Dwindra Sulistyoutomo
 */

#include "ros/ros.h"
#include <XmlRpcValue.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "mpu6050_ros/mpu6050_handler.h"


/**
 * @brief Create a Rotation Matrix object from rotation infos on the given rotation array.
 * 
 * @param rotation_array {XmlRpc::XmlRpcValue}  ROS Node Handler
 * @return Eigen::Matrix3f  Rotation Matrix (3x3)
 */
Eigen::Matrix3f CreateRotationMatrix (XmlRpc::XmlRpcValue rotation_array) {
  // Create array buffer for all rotation applied in order
  transformation::RotationInfo rotation_list[rotation_array.size()];

  ROS_INFO("MPU6050 is set with orientation:");
  int8_t i;
  for (i=0; i<rotation_array.size(); i++){
    // Get the rotation information
    transformation::RotationInfo rotation;

    for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator p=rotation_array[i].begin(); p!=rotation_array[i].end(); ++p) {
      // Get the axis name
      if (p->first == "axis") {
        rotation.axis = static_cast<std::string>(p->second);
      }
      // Get the rotation angle
      else if(p->first == "angle"){
        if (p->second.getType() == XmlRpc::XmlRpcValue::TypeInt) {
          rotation.angle = static_cast<double>(static_cast<int>(p->second));
        }
        else {
          rotation.angle = static_cast<double>(p->second);
        }
      }
    }
    ROS_INFO("%d. Rotation on %s %.1f degrees", i+1, rotation.axis.c_str(), rotation.angle);
    rotation_list[i] = rotation;
  }

  Eigen::Matrix3f M = transformation::CreateRotationMatrix(rotation_list, rotation_array.size());
  return M;
}

/**
 * @brief Publish static tf for the IMU model from the body
 * 
 * @param M {Eigen::Matrix3f}         Rotation matrix from parent_frame to child_frame
 * @param header_frame {std::string}  Parent frame ID (body)
 * @param child_frame {std::string}   Child frame ID (IMU model)
 */
void PublishImuStaticTf(Eigen::Matrix3f M, std::string parent_frame="imu", std::string child_frame="imu_model") {
  // Create tf broadcaster and tf msg
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;

  // Convert to tf2 quaternion
  tf2::Quaternion q;

  tf2::Matrix3x3 M_tf(M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2));
  M_tf.getRotation(q);

  transformStamped.transform.rotation = tf2::toMsg(q);

  // Publish the Tf
  br.sendTransform(transformStamped);
}

/**
 * @brief Publish the tf from the IMU data
 * 
 * @param msg {sensor_msgs::Imu} IMU data
 */
void PublishImuTf(sensor_msgs::Imu msg, std::string header_frame="world") {
  // Create tf broadcaster and tf msg
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = header_frame;
  transformStamped.child_frame_id = msg.header.frame_id;

  // Convert to tf2 quaternion
  tf2::Quaternion q;
  tf2::fromMsg(msg.orientation, q);

  // Filter noise from the sensor
  float ERROR_LIM = 0.001;
  if (abs(q.length()-1) > ERROR_LIM){
    // Do not publish the noise
    return;
  }

  // Normalize the quaternion, then convert to msg
  q.normalize();
  transformStamped.transform.rotation = tf2::toMsg(q);

  // Publish the Tf
  br.sendTransform(transformStamped);
}


/**
 * @brief main
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpu6050_node");
  ROS_INFO("MPU6050 node created");

  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  // ROS topic publishers
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

  // Get MPU6050 calibration data for offsets
  std::string filepath = std::getenv("HOME");
  filepath += "/.local/share/MPU6050/calibration.csv";
  ROS_INFO("Using calibration data from: %s", filepath.c_str());

  int16_t *offsets;
  offsets = mpu6050_conversion::GetCalibrationData(filepath);

  // Get dmp flag
  bool dmp;
  nh.param<bool>("dmp", dmp, false);
  ROS_INFO_STREAM("DMP flag: " << dmp);

  // Initialize the MPU6050 data handler.
  MPU6050Handler mpu_handler(&nh, offsets, DEFAULT_RATE, dmp);

  // Get rotation information from ROS parameter
  XmlRpc::XmlRpcValue config_rotation;
  nh.getParam("rotation", config_rotation);

  if (config_rotation.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    // Create Rotation Matrix from rotation information
    Eigen::Matrix3f M = CreateRotationMatrix(config_rotation);

    // Set Rotation Matrix for the current MPU6050
    mpu_handler.SetRotationMatrix(M);

    // Publish the static tf
    PublishImuStaticTf(M);
  }

  // Start getting MPU6050 Data. Background thread will update the data.
  mpu_handler.Start();
  
  while (ros::ok()) {
    // Publish IMU Data msg
    sensor_msgs::Imu msg = mpu_handler.GetImuMsg();   
    imu_pub.publish(msg);

    // Publish the TF
    PublishImuTf(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}