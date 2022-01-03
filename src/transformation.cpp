/**
 * @author  Dwindra Sulistyoutomo
 */

#include "mpu6050_ros/transformation.h"


namespace transformation
{

double sin_rounded (double x) {
  double y = sin(x);
  if (fabs(y)<=ZERO_ROUND) y=0;
  return y;
}
double cos_rounded (double x) {
  double y = cos(x);
  if (fabs(y)<=ZERO_ROUND) y=0;
  return y;
}

Eigen::Matrix3f CreateRotationMatrix (std::string axis, double angle) {
  // Convert angle to radians
  angle *= degToRad;

  double c = cos_rounded(angle);
  double s = sin_rounded(angle);

  // Create Rotation Matrix based on axis
  Eigen::Matrix3f M;
  if (axis == "x") {
    M <<  1, 0, 0,
          0, c, s,
          0,-s, c;
  }
  else if (axis == "y") {
    M <<  c, 0, -s,
          0, 1, 0,
          s, 0, c;
  }
  else if (axis == "z") {
    M <<  c, s, 0,
         -s, c, 0,
          0, 0, 1;
  }

  return M;
}

Eigen::Matrix3f CreateRotationMatrix (RotationInfo rotation) {
  return CreateRotationMatrix(rotation.axis, rotation.angle);
}

Eigen::Matrix3f CreateRotationMatrix (RotationInfo *rotation, int length) {
  int len;
  if (length <=0) {
    // Calculate length based on the given array
    len = *(&rotation + 1) - rotation;
  }
  else {
    len = length;
  }

  // Matrix containers
  Eigen::Matrix3f M[len];
  Eigen::Matrix3f M_final = Eigen::Matrix3f::Identity();

  for (int8_t i=0; i<len; i++) {
    M[i] = CreateRotationMatrix(rotation[i]);
    // Calculate the product
    M_final = M_final * M[i];
  }
  return M_final;
}

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