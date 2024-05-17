#include "uniform_spherical_discretisation.h"

namespace cnearest {


Matrix unif_grid_discr(int msph) {
    Matrix xp(6 * msph * msph, 3);
    double ds = 2.0 / msph;
    double c = 1.0;
    int icount = 0;
    double xi, et;
    for (int i = 0; i < msph; i++) {
        for (int j = 0; j < msph; j++) {
            xi = -1 + ds * (j + 0.5);
            et = -1 + ds * (i + 0.5);
            xp(icount, 0) = c;
            xp(icount, 1) = xi;
            xp(icount, 2) = et;
            //
            xp(icount + msph * msph, 0) = -xi;
            xp(icount + msph * msph, 1) = c;
            xp(icount + msph * msph, 2) = et;
            //
            xp(icount + 2 * msph * msph, 0) = xi;
            xp(icount + 2 * msph * msph, 1) = et;
            xp(icount + 2 * msph * msph, 2) = c;
            //
            xp(icount + 3 * msph * msph, 0) = xi;
            xp(icount + 3 * msph * msph, 1) = -et;
            xp(icount + 3 * msph * msph, 2) = -c;
            //
            xp(icount + 4 * msph * msph, 0) = xi;
            xp(icount + 4 * msph * msph, 1) = -c;
            xp(icount + 4 * msph * msph, 2) = et;
            //
            xp(icount + 5 * msph * msph, 0) = -c;
            xp(icount + 5 * msph * msph, 1) = -xi;
            xp(icount + 5 * msph * msph, 2) = et;
            //
            icount++;
        }
    }
    return xp;
}


Collection<Vector3> UnitCubeDiscretisation::positions() const {
    auto mat = unif_grid_discr(_msph);
    Collection<Vector3> result;

    // change each row of the matrix into a position in the discretisation
    for (int i = 0; i < mat.rows(); i++) {
        result.emplace_back(mat.row(i));
    }
    return result;
}

Collection<Vector3> UniformSphericalDiscretisation::positions() const {

    Collection<Vector3> result;

    UnitCubeDiscretisation cube(_msph);
    //rescale the unit cube to magnitude radius 
    for (auto& position: cube.positions()) {
        result.emplace_back(_radius * position.normalized());
    }
    return result;
}

}  // namespace cnearest