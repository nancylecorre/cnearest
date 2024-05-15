#pragma once
#include <map>
#include <memory>

#include "definitions.h"
#include "inearest_neighbout_tiebreaker.h"
#include "rigid_body.h"

namespace cnearest {

/// <summary>
/// Assigning a unique global index to each quadrature node in the collection of rigid bodies
/// Assigning a unique global index to each force node in the collection of rigid bodies
/// </summary>
class GlobalIndexAssignment {
   public:
    GlobalIndexAssignment(const Collection<RigidBody>& bodies);
    ~GlobalIndexAssignment() = default;

    /// <summary>
    /// Mapping from each quadrature node/force node to a global index
    /// </summary>
    const std::map<const QuadratureNode*, int>& quadrature_node_indicies() const;
    const std::map<const ForceNode*, int>& force_node_indicies() const;

    /// <summary>
    /// Global index assigned to quadrature node/force node
    /// </summary>
    int index(const QuadratureNode*) const;
    int index(const ForceNode*) const;
    int index(const QuadratureNode&) const;
    int index(const ForceNode&) const;

   private:
    std::map<const QuadratureNode*, int> _quadrature_node_indicies;
    std::map<const ForceNode*, int> _force_node_indicies;
};

}  // namespace cnearest