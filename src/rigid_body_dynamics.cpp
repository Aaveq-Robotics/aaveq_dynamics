#include "aaveq_dynamics/utils.hpp"

#include "aaveq_dynamics/rigid_body_dynamics.hpp"

namespace ADynamics
{

    Eigen::Matrix3d inertia_matrix(const std::vector<PointMass> &points)
    {
        auto inertia = [](double x, double y, double z) -> Eigen::Matrix3d
        {
            return Eigen::Matrix3d{{std::pow(y, 2) + std::pow(z, 2), -x * y, -x * z},
                                   {-x * y, std::pow(x, 2) + std::pow(z, 2), -y * z},
                                   {-x * z, -y * z, std::pow(x, 2) + std::pow(y, 2)}};
        };

        Eigen::Matrix3d I;
        for (auto p : points)
            I += p.m() * inertia(p.x(), p.y(), p.z());

        return I;
    }

    Eigen::Matrix<double, 6, 6> mass_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix)
    {
        Eigen::Matrix<double, 6, 6> mass_matrix;
        mass_matrix << mass * Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), inertia_matrix;

        return mass_matrix;
    }

    Eigen::Matrix<double, 6, 6> coriolis_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix, const Eigen::Vector<double, 6> &nu)
    {
        Eigen::Vector<double, 3> omega = nu.tail(3); // Get last 3 elements
        Eigen::Matrix<double, 6, 6> coriolis_matrix;
        coriolis_matrix << mass * skew_symmetric_matrix(omega), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), -skew_symmetric_matrix(inertia_matrix * omega);

        return coriolis_matrix;
    }

    Eigen::Matrix3d transformation_matrix(const Eigen::Vector3d &attitude)
    {
        double phi = attitude.x();
        double theta = attitude.y();

        return Eigen::Matrix3d{{1, sin(phi) * (sin(theta) / cos(theta)), cos(phi) * (sin(theta) / cos(theta))},
                               {0, cos(phi), -sin(phi)},
                               {0, sin(phi) / cos(theta), cos(phi) / cos(theta)}};
    }

    Eigen::Matrix<double, 6, 6> J_Theta(const Eigen::Vector<double, 6> &eta)
    {
        Eigen::Vector<double, 3> Omega = eta.tail(3); // Get last 3 elements
        Eigen::Matrix<double, 6, 6> J_Theta;
        J_Theta << rotation_matrix_eb(Omega), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), transformation_matrix(Omega);

        return J_Theta;
    }

    ADynamics::tubleStates rigid_body_dynamics(double timestep,
                                               const Eigen::Vector<double, 6> &tau,
                                               const Eigen::Vector<double, 6> &state_body,
                                               const Eigen::Vector<double, 6> &state_earth,
                                               const double &mass,
                                               const Eigen::Matrix3d &inertia_matrix,
                                               const Eigen::Matrix<double, 6, 6> &mass_matrix)

    {
        Eigen::Vector<double, 6> nu = state_body;
        Eigen::Vector<double, 6> eta = state_earth;

        // Rigid body dynamics
        Eigen::Vector<double, 6> nu_dot = matrix_inverse(mass_matrix) * tau - matrix_inverse(mass_matrix) * coriolis_matrix(mass, inertia_matrix, nu) * nu;
        nu += nu_dot * timestep;

        // Body-fixed frame to earth-fixed frame
        Eigen::Vector<double, 6> eta_dot = J_Theta(eta) * nu;
        eta += eta_dot * timestep;

        return std::make_tuple(nu, nu_dot, eta, eta_dot);
    }

} // namespace ADynamics