#pragma once
#include "idiscretisation.h"

namespace cnearest {
    /// <summary>
    /// Discretisation scheme from a set of points provided during construction
    /// </summary>
    class EmpiricalDiscretisation : public iDiscretisation {
       public:
        EmpiricalDiscretisation(const Collection<Vector3>& positions) : _positions(positions){};

        Collection<Vector3> positions() const override;

       private:
        Collection<Vector3> _positions;
    };
}
