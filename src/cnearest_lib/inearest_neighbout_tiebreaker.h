#pragma once
#include "definitions.h"
#include "force_node.h"
#include "quadrature_node.h"

namespace cnearest {

/// <summary>
/// An interface (interchangeable class), defining a means of tiebreaking neigbour nodes which are the same (or nearly the same) distance away
/// </summary>
class iNearestNeighboutTiebreaker {
   public:
    virtual ~iNearestNeighboutTiebreaker(){};

    virtual const ForceNode* nearest_neighbour(const QuadratureNode&, Collection<const ForceNode*> candidates) const = 0;
};
}  // namespace cnearest