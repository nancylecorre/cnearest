#pragma once
#include "idiscretisation.h"

namespace cnearest {
/// <summary>
/// Discretisation scheme giving points all around a sphere according to the fibonacci sphere algorithm
/// </summary>
class FibonacciSphericalDiscretisation : public iDiscretisation {
   public:
    FibonacciSphericalDiscretisation(double radius, unsigned int number_of_points)
        : _radius(radius), _number_of_points(number_of_points){};
    ~FibonacciSphericalDiscretisation() = default;

    Collection<Vector3> positions() const override;

   private:
    double _radius;
    unsigned int _number_of_points;
};
}  // namespace cnearest