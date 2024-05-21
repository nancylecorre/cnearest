#pragma once
#include "definitions.h"
#include "nearest_neighbour_results.h"

namespace cnearest {

    /// <summary>
    /// An interface (interchangeable class), defining a means of distributing between
    /// neighbour nodes which are the same (or nearly the same) distance away
    /// </summary>
    class iNearestNeighbourDistributer {
       public:
        virtual ~iNearestNeighbourDistributer(){};

        virtual NearestNeighbourResult nearest_neighbour_result(
            const QuadratureNode&, Collection<const ForceNode*> candidates) const = 0;
    };
}
