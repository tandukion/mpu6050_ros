/**
 * @author  Dwindra Sulistyoutomo
 */

#include <Eigen/Dense>


namespace transformation
{
/**
 * Transform the array of vector data in 3D with rotated coordinate frame based on the given rotation matrix.
 * 
 * @param rotation_matrix {Eigen::Matrix3f} Rotation Matrix for the coordinate frame
 * @param vector {float*}  Array of vector data
 */
void RotateFrame (Eigen::Matrix3f rotation_matrix, float vector[3]);
}