/**
 * @author  Dwindra Sulistyoutomo
 */

#include <Eigen/Dense>

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

Eigen::Matrix3f CreateRotationMatrix (std::string axis, double angle);
Eigen::Matrix3f CreateRotationMatrix (RotationInfo rotation);
Eigen::Matrix3f CreateRotationMatrix (RotationInfo *rotation);

/**
 * Transform the array of vector data in 3D with rotated coordinate frame based on the given rotation matrix.
 * 
 * @param rotation_matrix {Eigen::Matrix3f} Rotation Matrix for the coordinate frame
 * @param vector {float*}  Array of vector data
 */
void RotateFrame (Eigen::Matrix3f rotation_matrix, float vector[3]);
}