#pragma once
#include "definitions.h"
#include "force_node.h"
#include "quadrature_node.h"

namespace cnearest {

/// <summary>
/// An interface (interchangeable class), defining a means of tiebreaking neighbour nodes which are the same (or nearly the same) distance away
/// </summary>
class iNearestNeighbourTiebreaker {
   public:
    virtual ~iNearestNeighbourTiebreaker(){};

    virtual const ForceNode* nearest_neighbour(const QuadratureNode&, Collection<const ForceNode*> candidates) const = 0;
};
}  // namespace cnearest