#pragma once
#include "inearest_neighbour_distributer.h"

namespace cnearest {

/// <summary>
/// Distributer algorithm fully loading the first of the candidates
/// </summary>
class NearestNeighbourDistributerFirst : public iNearestNeighbourDistributer {
   public:
    ~NearestNeighbourDistributerFirst() = default;

    NearestNeighbourResult nearest_neighbour_result(
        const QuadratureNode&, Collection<const ForceNode*> candidates) const override {
        return NearestNeighbourResult(candidates[0]);
    }
};


/// <summary>
/// Distributer algorithm fully loading the last of the candidates
/// </summary>
class NearestNeighbourDistributerLast : public iNearestNeighbourDistributer {
   public:
    ~NearestNeighbourDistributerLast() = default;

    NearestNeighbourResult nearest_neighbour_result(
        const QuadratureNode&, Collection<const ForceNode*> candidates) const override {
        return NearestNeighbourResult(candidates.back());
    }
};
}  // namespace cnearest