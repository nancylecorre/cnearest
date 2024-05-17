#pragma once
#include <map>
#include <memory>
#include "definitions.h"
#include "rigid_body.h"

namespace cnearest {

    
/// <summary>
/// collection of force nodes with weights
/// </summary>
class NearestNeighbourResult {
   public:
    /// <summary>
    /// Single force node with a weight
    /// </summary>
    struct ForceNodeWithWeight {
        double weight;
        const ForceNode* force_node;
    };
    /// <summary>
    /// Single force node with weight one
    /// </summary>
    NearestNeighbourResult(const ForceNode* node);

    NearestNeighbourResult(const Collection<ForceNodeWithWeight>& result);
    
    auto begin() const { return _result.begin(); }
    auto end() const { return _result.end(); }

    const ForceNode* sole() const;

   private:
    const Collection<ForceNodeWithWeight> _result;
};

/// <summary>
/// Results of an investigation into the nearest neighbours of each quadrature node
/// Comes in the form of a mapping from each quadrature node to a collection of force nodes with weights
/// </summary>
class NearestNeighbourResults {
   public:

    NearestNeighbourResults(const std::map<const QuadratureNode*, NearestNeighbourResult>&
                                nearest_neighbour_mapping);
    ~NearestNeighbourResults() = default;

    /// <summary>
    /// Mapping from each quadrature node result
    /// </summary>
    const std::map<const QuadratureNode*, NearestNeighbourResult>&
    nearest_neighbour_mapping() const;

   private:
    std::map<const QuadratureNode*, NearestNeighbourResult>
        _nearest_neighbour_mapping;
};
}  // namespace cnearest