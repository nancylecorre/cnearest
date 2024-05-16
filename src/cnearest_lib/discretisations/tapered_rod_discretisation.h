#pragma once
#include "idiscretisation.h"

namespace cnearest {
/// <summary>
/// Discretisation scheme giving points all around a sphere according to the fibonacci sphere algorithm
/// </summary>
class TaperedRodDiscretisation : public iDiscretisation {
   public:
    TaperedRodDiscretisation(double radius, unsigned int number_of_points_z)
        : _radius(radius), _number_of_points_z(number_of_points_z){};
    ~TaperedRodDiscretisation() = default;

    Collection<Vector3> positions() const override;

   private:
    double _max_radius;
    unsigned int _number_of_points_z;
};
}  // namespace cnearest
