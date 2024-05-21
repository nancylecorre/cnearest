#pragma once
#include "idiscretisation.h"

namespace cnearest {
    /// <summary>
    /// Discretisation scheme giving a single point with defined global coordinates
    /// </summary>
    class SinglePointDiscretisation : public iDiscretisation {
       public:
        SinglePointDiscretisation() : _x(0), _y(0), _z(0){};
        SinglePointDiscretisation(double x, double y, double z) : _x(x), _y(y), _z(z){};
        ~SinglePointDiscretisation() = default;

        Collection<Vector3> positions() const override { return {{_x, _y, _z}}; }

       private:
        double _x, _y, _z;
    };
}