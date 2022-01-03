/**
 * @author  Dwindra Sulistyoutomo
 */

#include <Eigen/Dense>

#include "math_3d.h"

#define ZERO_ROUND 1e-07

namespace transformation
{
const float degToRad = M_PI/180.0;

// rounding sine cosine
double sin_rounded (double x);
double cos_rounded (double x);

struct RotationInfo {
  std::string axis; // name of the rotation axis
  double angle;     // rotation angle in degrees
};

/**
 * Create a rotation matrix from the given information about the axis and rotation angle
 * 
 * @param axis {string}  name of the axis of the rotation
 * @param angle {double}      rotation angle (in degree)
 * @return {Eigen::Matrix3f}
 */
Eigen::Matrix3f CreateRotationMatrix (std::string axis, double angle);
/**
 * Create a rotation matrix from the given rotation info
 * 
 * @param rotation {RotationInfo}  rotation info
 * @return {Eigen::Matrix3f}
 */
Eigen::Matrix3f CreateRotationMatrix (RotationInfo rotation);
/**
 * Create a rotation matrix from the given list of rotation info.
 * The final rotation matrix is the product of rotation info (in order)
 * 
 * @param rotation {RotationInfo*}  Array of rotation info
 * @param length {int}              Length of the array. 0 for dynamically check the length
 * @return {Eigen::Matrix3f}
 */
Eigen::Matrix3f CreateRotationMatrix (RotationInfo *rotation, int length=0);

/**
 * Create a Rotation Matrix object from a Quaternion object
 * ref: https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
 * 
 * @param q {Quaternion}    Quaternion
 * @return {Eigen::Matrix3f} 
 */
Eigen::Matrix3f CreateRotationMatrix (Quaternion q);

/**
 * Create a Quaternion object from a Rotation Matrix object
 * ref:https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
 * 
 * @param M {Eigen::Matrix3f} Rotation Matrix
 * @return {Quaternion} 
 */
Quaternion CreateQuaternion (Eigen::Matrix3f M);

/**
 * Transform the array of vector data in 3D with rotated coordinate frame based on the given rotation matrix.
 * 
 * @param rotation_matrix {Eigen::Matrix3f} Rotation Matrix for the coordinate frame
 * @param vector {float*}  Array of vector data
 */
void RotateFrame (Eigen::Matrix3f rotation_matrix, float vector[3]);

/**
 * Transform the Vector in 3D with rotated coordinate frame based on the given rotation matrix.
 * 
 * @param rotation_matrix {Eigen::Matrix3f} Rotation Matrix for the coordinate frame
 * @param vector {Vector}  Vector data
 */
void RotateFrame (Eigen::Matrix3f rotation_matrix, Vector *vector);

/**
 * Transform the Quaternion with the given rotation matrix.
 * 
 * @param rotation_matrix {Eigen::Matrix3f} Rotation Matrix
 * @param q  {Quaternion} Quaternion data
 */
void RotateFrame (Eigen::Matrix3f rotation_matrix, Quaternion *q);
}