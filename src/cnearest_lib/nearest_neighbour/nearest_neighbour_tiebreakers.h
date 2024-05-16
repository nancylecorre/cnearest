#pragma once
#include "inearest_neighbour_tiebreaker.h"

namespace cnearest {

/// <summary>
/// Tiebreaker algorithm taking the first of the candidates
/// </summary>
class NearestNeighbourTiebreakerFirst : public iNearestNeighbourTiebreaker {
   public:
    ~NearestNeighbourTiebreakerFirst() = default;

    const ForceNode* nearest_neighbour(
        const QuadratureNode&, Collection<const ForceNode*> candidates) const override {
        return candidates[0];
    }
};


/// <summary>
/// Tiebreaker algorithm taking the last of the candidates
/// </summary>
class NearestNeighbourTiebreakerLast : public iNearestNeighbourTiebreaker {
   public:
    ~NearestNeighbourTiebreakerLast() = default;

    const ForceNode* nearest_neighbour(
        const QuadratureNode&, Collection<const ForceNode*> candidates) const override {
        return candidates.back();
    }
};
}  // namespace cnearest