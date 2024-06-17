#pragma once
#include "idiscretisation.h"

namespace cnearest {

    /// <summary>
    /// Discretisation scheme giving points on the unit cube
    /// Each face of the cube is given an n*n grid which doesn't reach the edges.
    /// This results in 6*n*n nodes, it never reaches corners or edges to avoid duplicate
    /// nodes Mostly used to prepare the uniform spherical distribution
    /// </summary>
    class UnitCubeDiscretisation : public iDiscretisation {
       public:
        UnitCubeDiscretisation(unsigned int msph) : _msph(msph){};

        Collection<Vector3> positions() const override;

       private:
        unsigned int _msph;
    };

    /// <summary>
    /// Discretisation scheme giving points all around a sphere
    /// Achieved by discretising the 6 faces of a cube, then projecting the resultant
    /// nodes onto the surface of a sphere This results in 6*n*n nodes See Figure 7. of
    /// Proc.R.Soc. A(2009) 465,3605-3626
    /// </summary>
    class UniformSphericalDiscretisation : public iDiscretisation {
       public:
        UniformSphericalDiscretisation(double radius, unsigned int msph) : _radius(radius), _msph(msph){};

        Collection<Vector3> positions() const override;

       private:
        double _radius;
        unsigned int _msph;
    };
}
