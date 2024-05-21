#pragma once
#include "definitions.h"

namespace cnearest {

    /// <summary>
    /// An interface (interchangeable class), describing a discretisation scheme
    /// These can be used to create either force discretisations or quadrature
    /// discretisations of a rigid body
    /// </summary>
    class iDiscretisation {
       public:
        virtual ~iDiscretisation(){};

        virtual Collection<Vector3> positions() const = 0;
    };
}