#pragma once
#include "inearest_neighbout_tiebreaker.h"

namespace cnearest {

/// <summary>
/// Tiebreaker algorithm taking the first of the candidates
/// </summary>
class NearestNeighboutTiebreakerFirst : public iNearestNeighboutTiebreaker {
   public:
    ~NearestNeighboutTiebreakerFirst() = default;

    const ForceNode* nearest_neighbour(
        const QuadratureNode&, Collection<const ForceNode*> candidates) const override {
        return candidates[0];
    }
};


/// <summary>
/// Tiebreaker algorithm taking the last of the candidates
/// </summary>
class NearestNeighboutTiebreakerLast : public iNearestNeighboutTiebreaker {
   public:
    ~NearestNeighboutTiebreakerLast() = default;

    const ForceNode* nearest_neighbour(
        const QuadratureNode&, Collection<const ForceNode*> candidates) const override {
        return candidates.back();
    }
};
}  // namespace cnearest