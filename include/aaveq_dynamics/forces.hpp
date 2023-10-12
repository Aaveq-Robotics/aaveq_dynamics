#ifndef FORCES_H_
#define FORCES_H_

#include <Eigen/Dense>

namespace ADynamics
{

    Eigen::Vector3d drag(const Eigen::Vector3d &v, const double &A, const double &C, const double &rho);

} // namespace ADynamics

#endif // FORCES_H_