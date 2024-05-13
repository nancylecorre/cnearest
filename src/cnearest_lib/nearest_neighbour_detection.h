#pragma once
#include "definitions.h"
#include <map>
#include <memory>
#include "rigid_body.h"
#include "inearest_neighbout_tiebreaker.h"

namespace cnearest {

class NearestNeighbourResults {
   public:
    NearestNeighbourResults(const std::map<const QuadratureNode*, const ForceNode*>&
                                nearest_neighbour_mapping);
    ~NearestNeighbourResults() = default;
    const std::map<const QuadratureNode*, const ForceNode*>& nearest_neighbour_mapping();

   private:
    std::map<const QuadratureNode*, const ForceNode*> _nearest_neighbour_mapping;
};

class NearestNeighbourDetectionAlgorithm {
   public:
    NearestNeighbourDetectionAlgorithm(std::unique_ptr<iNearestNeighboutTiebreaker> tiebreaker = nullptr, double tie_shell_thickness=0.0);
    ~NearestNeighbourDetectionAlgorithm() = default;

    NearestNeighbourResults nearest_neighbours(const Collection<RigidBody>& bodies) const;

   private:
    std::unique_ptr<iNearestNeighboutTiebreaker> _tiebreaker;
    double _tie_shell_thickness;

    void populate_nearest_neighbours(
        const QuadratureNode& node, const RigidBody& body,
        std::map<const QuadratureNode*, const ForceNode*>& results) const;
};

}  // namespace cnearest