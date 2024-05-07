#include "fibonacci_sphere_discretisation.h"

namespace cnearest {


Collection<Vector3> FibonacciSphericalDescretisation::positions() const {
    Collection<Vector3> result;

    const double phi = M_PI * (std::sqrt(5.0) - 1.0);  // golden angle in radians

    for (int i = 0; i < _number_of_points; i++) {
        double y = 1.0 - (i / (_number_of_points - 1)) * 2.0;  // y goes from 1 to - 1
        double radius = std::sqrt(1 - y * y);         // radius of 2d circle at at y-slice
        double theta = phi * i;                       // golden angle increment
        double x = std::cos(theta) * radius;
        double z = std::sin(theta) * radius;

        result.push_back({_radius * x, _radius * y, _radius * z});
    }
    return result;
}
}  // namespace cnearest