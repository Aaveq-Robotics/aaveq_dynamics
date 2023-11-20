#include "aaveq_dynamics/forces.hpp"

#include <iostream>

namespace ADynamics
{
    Eigen::Vector3d drag(const Eigen::Vector3d &v, const double &A, const double &C, const double &rho)
    {
        // Translation
        double force_mag = (std::pow(v.norm(), 2) * A * C * rho) / 2;

        Eigen::Vector3d force{0.0, 0.0, 0.0};
        if (v.norm() != 0.0)
            force = (-1) * v.normalized() * force_mag;

        return force;
    }

} // namespace ADynamics