#include "regularised_stokeslet.h"

namespace cnearest {

    Matrix3 RegularisedStokeslet::stokeslet_tensor(const ForceNode& force_node,
                                                   const QuadratureNode& quadrature_node,
                                                   double epsilon) {
        const Vector3& x = quadrature_node.position();
        const Vector3& y = force_node.position();  // also called x_0 in some papers
        return stokeslet_tensor(x - y, epsilon);
    }
    Vector3 RegularisedStokeslet::pressure(const ForceNode& force_node,
                                                  const QuadratureNode& quadrature_node,
                                                  double epsilon) {
        const Vector3& x = quadrature_node.position();
        const Vector3& y = force_node.position();  // also called x_0 in some papers
        return pressure(x - y, epsilon);
    }

    std::array<Matrix3, 3> RegularisedStokeslet::stress_tensor(
        const ForceNode& force_node, const QuadratureNode& quadrature_node,
        double epsilon) {
        const Vector3& x = quadrature_node.position();
        const Vector3& y = force_node.position();  // also called x_0 in some papers
        return stress_tensor(x - y, epsilon);
    }

    Matrix3 RegularisedStokeslet::stokeslet_tensor(const Vector3& offset,
                                                   double epsilon) {
        const double R2 = offset.squaredNorm();
        const double regularised_R2 = R2 + epsilon * epsilon;
        const Matrix3 isotropic = Matrix3::Identity() * (R2 + 2.0 * epsilon * epsilon) *
                                  std::pow(regularised_R2, -1.5);
        const Matrix3 dyadic = offset * offset.transpose() * std::pow(regularised_R2, -1.5);
        return isotropic + dyadic;
    }
    Vector3 RegularisedStokeslet::pressure(const Vector3& offset, double epsilon) {
        const double R2 = offset.squaredNorm();
        const double regularised_R2 = R2 + epsilon * epsilon;
        return offset * (2 * R2 + 5 * epsilon * epsilon) * std::pow(regularised_R2, -2.5); 
    }

    static inline double stress_tensor_correction(const Vector3& offset,double base_scale, double correction_scale, int i, int j,
                                    int k) {
        double base = offset(i) * offset(j) * offset(k);
        
        double correction = 0;
        if (j == k) correction += offset(i);
        if (i == k) correction += offset(j);
        if (i == j) correction += offset(k);
        return base_scale*base + correction_scale * correction;
    }

    std::array<Matrix3, 3> RegularisedStokeslet::stress_tensor(const Vector3& offset,
                                                               double epsilon) {
        const double R2 = offset.squaredNorm();
        const double regularised_R2 = R2 + epsilon * epsilon;
        const double base_scale = -6 * std::pow(regularised_R2, -2.5); 
        
        const double correction_scale =-3*epsilon*epsilon * std::pow(regularised_R2, -2.5); 

        std::array<Matrix3, 3> result;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    result[k](i, j) = stress_tensor_correction(
                        offset, base_scale, correction_scale, i, j, k);
                }
            }
        }

        return result;
    }
}
