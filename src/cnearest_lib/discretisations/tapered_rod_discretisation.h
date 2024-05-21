#pragma once
#include "idiscretisation.h"

namespace cnearest {
    /// <summary>
    /// Discretisation scheme giving points all around a sphere according to the fibonacci
    /// sphere algorithm
    /// </summary>
    class TaperedRodDiscretisation : public iDiscretisation {
       public:
        TaperedRodDiscretisation(double radius, unsigned int number_of_points,
                                 unsigned int number_of_points_azi)
            : _radius(radius),
              _number_of_points(number_of_points),
              _number_of_points_azi(number_of_points_azi){};
        ~TaperedRodDiscretisation() = default;

        Collection<Vector3> positions() const override;

       private:
        double _radius;
        unsigned int _number_of_points;
        unsigned int _number_of_points_azi;
    };
}
