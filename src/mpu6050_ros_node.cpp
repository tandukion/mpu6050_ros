/**
 * @author  Dwindra Sulistyoutomo
 */

#include "ros/ros.h"
#include <XmlRpcValue.h>

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

  // Initialize the MPU6050 data handler.
  MPU6050Handler mpu_handler(&nh, offsets);

  // Get rotation information from ROS parameter
  XmlRpc::XmlRpcValue config_rotation;
  nh.getParam("rotation", config_rotation);

  if (config_rotation.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    // Create array buffer for all rotation applied in order
    transformation::RotationInfo rotation_list[config_rotation.size()];

    ROS_INFO("MPU6050 is set with orientation:");
    int8_t i;
    for (i=0; i<config_rotation.size(); i++){
      // Get the rotation information
      transformation::RotationInfo rotation;

      for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator p=config_rotation[i].begin(); p!=config_rotation[i].end(); ++p) {
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
    
    // Create Rotation Matrix from rotation information
    Eigen::Matrix3f M = transformation::CreateRotationMatrix(rotation_list);

    // Set Rotation Matrix for the current MPU6050
    mpu_handler.SetRotationMatrix(M);
  }

  // Start getting MPU6050 Data. Background thread will update the data.
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