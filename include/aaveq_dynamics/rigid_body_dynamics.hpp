#ifndef RIGID_BODY_DYNAMICS_H_
#define RIGID_BODY_DYNAMICS_H_

#include <Eigen/Dense>

namespace ADynamics
{
    // Add .m() accesser to Eigen::Vector4d, equivalent to operator[](3)
    class PointMass : public Eigen::Vector4d
    {
    public:
        double m() const { return (*this)[3]; }
    };

    Eigen::Matrix3d inertia_matrix(const std::vector<PointMass> &points);

    Eigen::Matrix<double, 6, 6> mass_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix);

    Eigen::Matrix<double, 6, 6> coriolis_matrix(const double &mass, const Eigen::Matrix3d &inertia_matrix, const Eigen::Vector<double, 6> &nu);

    Eigen::Matrix3d transformation_matrix(const Eigen::Vector3d &attitude);

    Eigen::Matrix<double, 6, 6> J_Theta(const Eigen::Vector<double, 6> &eta);

    typedef std::tuple<Eigen::Vector<double, 6>, Eigen::Vector<double, 6>, Eigen::Vector<double, 6>, Eigen::Vector<double, 6>> tubleStates;
    tubleStates rigid_body_dynamics(double timestep,
                                    const Eigen::Vector<double, 6> &tau,
                                    const Eigen::Vector<double, 6> &state_body,
                                    const Eigen::Vector<double, 6> &state_earth,
                                    const double &mass,
                                    const Eigen::Matrix3d &inertia_matrix,
                                    const Eigen::Matrix<double, 6, 6> &mass_matrix);

} // namespace ADynamics

#endif // RIGID_BODY_DYNAMICS_H_