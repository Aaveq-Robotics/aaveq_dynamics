#ifndef UTILS_H_
#define UTILS_H_

#include <Eigen/Dense>

namespace ADynamics
{

    Eigen::Matrix3d skew_symmetric_matrix(const Eigen::Vector3d &v);

    Eigen::Matrix<double, 6, 6> matrix_inverse(const Eigen::Matrix<double, 6, 6> &matrix);

    Eigen::Matrix3d rotation_matrix_eb(const Eigen::Vector3d &attitude);

} // namespace ADynamics

#endif // UTILS_H_