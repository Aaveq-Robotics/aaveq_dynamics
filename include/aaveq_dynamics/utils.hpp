#ifndef UTILS_H_
#define UTILS_H_

#include <Eigen/Dense>

namespace ADynamics
{

    Eigen::Matrix3d skew_symmetric_matrix(const Eigen::Vector3d &v);

    Eigen::Matrix<double, 6, 6> matrix_inverse(const Eigen::Matrix<double, 6, 6> &matrix);

    Eigen::Matrix3d rotation_matrix_eb(const Eigen::Vector3d &attitude);

    // Template functions
    // non-specialized template must be visible to a translation unit that uses it, hence it's implemented in the header
    template <typename EigenVec>
    std::vector<Eigen::Vector3d> body_to_earth(const std::vector<EigenVec> &points_body, const Eigen::Vector3d &position_earth, const Eigen::Vector3d &attitude_earth)
    {
        std::vector<Eigen::Vector3d> points_earth;
        for (EigenVec p_body : points_body)
        {
            Eigen::Vector3d p_earth = position_earth + ADynamics::rotation_matrix_eb(attitude_earth) * p_body.head(3);
            points_earth.push_back(p_earth);
        }

        return points_earth;
    }
} // namespace ADynamics

#endif // UTILS_H_