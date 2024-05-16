#include "tapered_rod_discretisation.h"

namespace cnearest {


Collection<Vector3> TaperedRodDiscretisation::positions() const{
    Collection<Vector3> result;

    for (int i = 0; i < _number_of_points_z; i++) {
        double z = -1.0 + (i / (_number_of_points_z - 1) ) * 2.0;
        double Rscl = 1.0 - 0.95 * ( exp( -200.0 * pow(z,2) ) + exp( -200.0 * pow(1.0-z,2)) ); // rod narrows at each end. The specific numbers here could be adjustable constants
        int npts = std::ceil(_max_number_of_points*Rscl); // fewer azimuthal points where rod is narrower
        double r = Rscl * _max_radius;
        for (int j = 0; j < npts, j++){
            double theta = 2.0 * M_PI * j * ( 1.0 / npts );
            double x = r * std::cos(theta);
            double y = r * std::sin(theta);

            result.push_back({x, y, z});
        }
    }
}
} // namespace cnearest
