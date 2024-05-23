#pragma once
#include <Eigen/Dense>
#include <memory>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

namespace cnearest {

    typedef Eigen::MatrixXd Matrix;
    typedef Eigen::Matrix3d Matrix3;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::Vector3d Vector3;
    template <typename T>
    using Collection = std::vector<T>;
}