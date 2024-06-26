#include "tapered_rod_discretisation.h"

namespace cnearest {

    Collection<Vector3> TaperedRodDiscretisation::positions() const {
        Collection<Vector3> result;

        const double phi = M_PI * (std::sqrt(5.0) - 1.0);  // golden angle in radians

        for (size_t i = 0; i < _number_of_points; i++) {
            double z = -1.0 + i * 2.0 / (_number_of_points - 1);
            double Rscl = 1.0 - 0.95 * ( exp( -200.0 * pow(z,2) ) + exp( -200.0 * pow(1.0-z,2)) ); // rod narrows at each end. The specific numbers here could be adjustable constants
            int npts = (int)std::ceil(_number_of_points_azi*Rscl); // fewer azimuthal points where rod is narrower
            double r = Rscl * _radius;
            for (int j = 0; j < npts; j++){
                double y = 1.0 - (i / (_number_of_points - 1)) * 2.0;  // y goes from 1 to - 1
                double radius = std::sqrt(1 - y * y);  // radius of 2d circle at at y-slice
                double theta = phi * i;                // golden angle increment
                double x = std::cos(theta) * radius;
                result.push_back({_radius * x, _radius * y, _radius * z});
            }
        }
        return result;
    }
}
