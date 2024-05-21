#include "uniform_spherical_discretisation.h"

namespace cnearest {

    Matrix sph_projector(const Matrix& xp, double a) {
        Matrix xs(xp.rows(), 3);
        int mpts = xp.rows();
        double r, th, phi;
        for (int j = 0; j < mpts; j++) {
            r = xp(j, 0) * xp(j, 0) + xp(j, 1) * xp(j, 1) + xp(j, 2) * xp(j, 2);
            th = acos(xp(j, 2) / r);
            //
            if (xp(j, 1) == 0) phi = -acos((xp(j, 0) + 1.0e-9) / abs(xp(j, 0) + 1.0e-9));
            if (xp(j, 1) > 0)
                phi = acos((xp(j, 0) + 1.0e-9) /
                           pow(pow(xp(j, 0) + 1.0e-9, 2) + pow(xp(j, 1), 2), 0.5));
            if (xp(j, 1) < 0)
                phi = acos((xp(j, 0) + 1.0e-9) /
                           pow(pow(xp(j, 0) + 1.0e-9, 2) + pow(xp(j, 1), 2), 0.5)) +
                      2.0 * M_PI;
            //
            xs(j, 0) = a * sin(th) * cos(phi);
            xs(j, 1) = a * sin(th) * sin(phi);
            xs(j, 2) = a * cos(th);
        }
        return xs;
    }

    Matrix unif_sphere_discr(double a, int msph) {
        Matrix xp(6 * msph * msph, 3);
        double ds = 2.0 / msph;
        double c = 1.0;
        int icount = 0;
        double xi, et;
        for (int i = 0; i < msph; i++) {
            for (int j = 0; j < msph; j++) {
                xi = -1 + ds * (j - 0.5);
                et = -1 + ds * (i - 0.5);
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
        return sph_projector(xp, a);
    }

    Collection<Vector3> UniformSphericalDiscretisation::positions() const {
        auto mat = unif_sphere_discr(_radius, _msph);
        Collection<Vector3> result;

        // change each row of the matrix into a position in the discretisation
        for (int i = 0; i < mat.rows(); i++) {
            result.emplace_back(mat.row(i));
        }
        return result;
    }
}