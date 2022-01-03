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

Eigen::Matrix3f CreateRotationMatrix (Quaternion q) {
  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  double sqw = q.w*q.w;
  double sqx = q.x*q.x;
  double sqy = q.y*q.y;
  double sqz = q.z*q.z;

  // invs (inverse square length) is only required if quaternion is not already normalised
  double invs = 1 / (sqx + sqy + sqz + sqw);
  m00 = ( sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz =1/invs*invs
  m11 = (-sqx + sqy - sqz + sqw) * invs;
  m22 = (-sqx - sqy + sqz + sqw) * invs;
  
  double tmp1 = q.x*q.y;
  double tmp2 = q.z*q.w;
  m10 = 2.0 * (tmp1 + tmp2) * invs;
  m01 = 2.0 * (tmp1 - tmp2) * invs;
  
  tmp1 = q.x*q.z;
  tmp2 = q.y*q.w;
  m20 = 2.0 * (tmp1 - tmp2) * invs;
  m02 = 2.0 * (tmp1 + tmp2) * invs;
  tmp1 = q.y*q.z;
  tmp2 = q.x*q.w;
  m21 = 2.0 * (tmp1 + tmp2) * invs;
  m12 = 2.0 * (tmp1 - tmp2) * invs;

  Eigen::Matrix3f M;
  M <<  m00, m01, m02,
        m10, m11, m12,
        m20, m21, m22;
  
  return M;
}

Quaternion CreateQuaternion (Eigen::Matrix3f M) {
  Quaternion q;
  float trace = M(0,0) + M(1,1) + M(2,2) + 1;
  if (trace > 0) {
    float s = 0.5 / sqrt(trace);
    q.w = 0.25 / s;
    q.x = (M(2,1) - M(1,2)) * s;
    q.y = (M(0,2) - M(2,0)) * s;
    q.z = (M(1,0) - M(0,1)) * s;
  }
  else {
    if ((M(0,0) > M(1,1)) && (M(0,0) > M(2,2))) { 
      float s = sqrt(1.0 + M(0,0) - M(1,1) - M(2,2)) * 2; // S=4*qx 
      q.w = (M(2,1) - M(1,2)) / s;
      q.x = 0.25 * s;
      q.y = (M(0,1) + M(1,0)) / s; 
      q.z = (M(0,2) + M(2,0)) / s; 
    } else if (M(1,1) > M(2,2)) { 
      float s = sqrt(1.0 + M(1,1) - M(0,0) - M(2,2)) * 2; // S=4*qy
      q.w = (M(0,2) - M(2,0)) / s;
      q.x = (M(0,1) + M(1,0)) / s; 
      q.y = 0.25 * s;
      q.z = (M(1,2) + M(2,1)) / s; 
    } else { 
      float s = sqrt(1.0 + M(2,2) - M(0,0) - M(1,1)) * 2; // S=4*qz
      q.w = (M(1,0) - M(0,1)) / s;
      q.x = (M(0,2) + M(2,0)) / s;
      q.y = (M(1,2) + M(2,1)) / s;
      q.z = 0.25 * s;
    }
  }
  q.Normalize();
  return q;
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

void RotateFrame (Eigen::Matrix3f rotation_matrix, Vector *vector) {
  // Convert the Vector to Vector3f
  Eigen::Vector3f v(vector->x, vector->y, vector->z);

  // Apply the rotation matrix
  v = rotation_matrix * v;

  // Return the vector with the new coordinate frame
  vector->x = v(0); 
  vector->y = v(1);
  vector->z = v(2);
}

void RotateFrame (Eigen::Matrix3f rotation_matrix, Quaternion *q) {
  // Convert the Quaternion to Rotation Matrix
  Eigen::Matrix3f M = CreateRotationMatrix(*q);

  // Apply the rotation matrix
  M = rotation_matrix * M;

  *q = CreateQuaternion(M);
}

}