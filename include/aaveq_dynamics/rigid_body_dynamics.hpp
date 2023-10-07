#ifndef RIGID_BODY_DYNAMICS_H_
#define RIGID_BODY_DYNAMICS_H_

#include <Eigen/Dense>

namespace ADynamics
{
    struct PointMass
    {
        double m;
        double x;
        double y;
        double z;
    };

    Eigen::Matrix3d inertia_matrix(const std::vector<PointMass> &points);

    Eigen::Matrix<double, 6, 6> mass_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix);

    Eigen::Matrix<double, 6, 6> coriolis_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix, const Eigen::Vector<double, 6> &nu);

    Eigen::Matrix3d transformation_matrix(const Eigen::Vector3d &attitude);

    Eigen::Matrix<double, 6, 6> J_Theta(const Eigen::Vector<double, 6> &eta);

    void rigid_body_dynamics(double timestep,
                             const Eigen::Vector<double, 6> &tau,
                             const Eigen::Matrix<double, 6, 6> &mass_matrix,
                             const Eigen::Vector<double, 6> &state_body,
                             const Eigen::Vector<double, 6> &state_earth);

} // namespace ADynamics

#endif // RIGID_BODY_DYNAMICS_H_