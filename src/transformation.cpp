/**
 * @author  Dwindra Sulistyoutomo
 */

#include "mpu6050_ros/transformation.h"


namespace transformation
{
void RotateFrame (Eigen::Matrix3f rotation_matrix, float vector[3]) {
  // Create the vectors of gyro and accel with the current coordinate frame
  Eigen::Vector3f v(vector[0],vector[1],vector[2]);

  // Apply the rotation matrix
  v = rotation_matrix * v;

  // Return the vectors of gyro and accel with the new coordinate frame
  int8_t i;
  for (i=0;i<3;i++){
    vector[i] = v(i);
  }
}
}