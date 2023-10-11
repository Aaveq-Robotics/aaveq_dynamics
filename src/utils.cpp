#include "aaveq_dynamics/utils.hpp"

namespace ADynamics
{
    Eigen::Matrix3d skew_symmetric_matrix(const Eigen::Vector3d &v)
    {
        return Eigen::Matrix3d{{0, -v.z(), v.y()}, {v.z(), 0, -v.x()}, {-v.y(), v.x(), 0}};
    }

    Eigen::Matrix<double, 6, 6> matrix_inverse(const Eigen::Matrix<double, 6, 6> &matrix)
    {
        const float epsilon = 1e-6f; // Small tolerance value

        if (matrix.determinant() > epsilon)
            return matrix.inverse();
        else
            return matrix.completeOrthogonalDecomposition().pseudoInverse();
    }

    Eigen::Matrix3d rotation_matrix_eb(const Eigen::Vector3d &attitude)
    {
        double phi = attitude.x();
        double theta = attitude.y();
        double psi = attitude.z();

        // Rotation of earth-fixed (NED) frame with respect to body-fixed frame
        Eigen::Matrix3d Rx{{1, 0, 0}, {0, cos(phi), -sin(phi)}, {0, sin(phi), cos(phi)}};
        Eigen::Matrix3d Ry{{cos(theta), 0, sin(theta)}, {0, 1, 0}, {-sin(theta), 0, cos(theta)}};
        Eigen::Matrix3d Rz{{cos(psi), -sin(psi), 0}, {sin(psi), cos(psi), 0}, {0, 0, 1}};

        return Rz * Ry * Rx;
    }

} // namespace ADynamics