#pragma once
#include "definitions.h"

namespace cnearest {

    class iDescretisation {
        virtual ~iDescretisation(){};

        virtual Collection<Vector3> relative_offsets() = 0;
    };
}