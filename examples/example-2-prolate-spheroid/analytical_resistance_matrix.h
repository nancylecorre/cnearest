#include  "definitions.h"

cnearest::Matrix analytic_grand_resistance_matrix(double radius_x, double radius_y_z) {
    cnearest::Matrix result = cnearest::Matrix::Zero(6, 6);

    double e = std::sqrt(radius_x * radius_x - radius_y_z * radius_y_z) / radius_x;
    double e_sqrd = std::pow(e, 2);
    double e_cubd = std::pow(e, 3);

    cnearest::Vector3 d = {1, 0, 0};

    double L = log((1 + e) / (1 - e));

    double XA = 8.0 / 3 * e_cubd / (-2 * e + (1 + e_sqrd) * L);
    double YA = 16.0 / 3 * e_cubd / (2 * e + (3 * e_sqrd - 1) * L);
    double XC = 4.0 / 3 * e_cubd * (1 - e_sqrd) / (2 * e - (1 - e_sqrd) * L);
    double YC = 4.0 / 3 * e_cubd * (2 - e_sqrd) / (-2 * e + (1 + e_sqrd) * L);

    cnearest::Matrix3 A_an;
    A_an << XA, 0, 0, 0, YA, 0, 0, 0, YA;
    A_an *= (6 * M_PI * radius_x);

    cnearest::Matrix3 B_an = cnearest::Matrix3::Zero();

    cnearest::Matrix3 C_an;
    C_an << XC, 0, 0, 0, YC, 0, 0, 0, YC;
    C_an *= (8 * M_PI * pow(radius_x, 3));

    result.block<3, 3>(0, 0) = A_an;
    result.block<3, 3>(0, 3) = B_an;
    result.block<3, 3>(3, 0) = B_an;
    result.block<3, 3>(3, 3) = C_an;

    return result;
}