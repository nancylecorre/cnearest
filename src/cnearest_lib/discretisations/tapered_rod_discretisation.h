#pragma once
#include "idiscretisation.h"

namespace cnearest {
    /// <summary>
    /// Discretisation scheme giving points all tapered rod
    /// </summary>
    class TaperedRodDiscretisation : public iDiscretisation {
       public:
        TaperedRodDiscretisation(double max_radius, unsigned int number_of_points_z,
                                 unsigned int max_number_of_points)
            : _max_radius(max_radius),
              _number_of_points_z(number_of_points_z),
              _max_number_of_points(max_number_of_points){};
        ~TaperedRodDiscretisation() = default;

        Collection<Vector3> positions() const override;

       private:
        double _max_radius;
        unsigned int _number_of_points_z;
        unsigned int _max_number_of_points;
    };
}
