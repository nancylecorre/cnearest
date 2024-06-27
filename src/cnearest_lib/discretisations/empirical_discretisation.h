#pragma once
#include "idiscretisation.h"

namespace cnearest {
    /// <summary>
    /// Discretisation scheme from a set of points provided during construction
    /// </summary>
    class EmpiricalDiscretisation : public iDiscretisation {
       public:
        EmpiricalDiscretisation(const Collection<Vector3>& positions) : _positions(positions){};

        // This constructor may prevent one copy of the collection if used
        EmpiricalDiscretisation(Collection<Vector3>&& positions) : _positions(std::move(positions)){};

        Collection<Vector3> positions() const override;

       private:
        Collection<Vector3> _positions;
    };
}
