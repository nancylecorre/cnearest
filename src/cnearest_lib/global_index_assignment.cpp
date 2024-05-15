#include "global_index_assignment.h"

namespace cnearest {

    GlobalIndexAssignment::GlobalIndexAssignment(const Collection<RigidBody>& bodies) {
    int quadrature_node_count = 0;
    int force_node_count = 0;
    for (auto& body : bodies) {
        for (auto& force_node : body.force_nodes().nodes()) {
            _force_node_indicies.insert({&force_node, force_node_count});
            force_node_count++;
        }
        for (auto& quadrature_node : body.quadrature_nodes().nodes()) {
            _quadrature_node_indicies.insert({&quadrature_node, quadrature_node_count});
            quadrature_node_count++;
        }
    }
}


const std::map<const QuadratureNode*, int>&
GlobalIndexAssignment::quadrature_node_indicies() const {
    return _quadrature_node_indicies;
}
const std::map<const ForceNode*, int>&
GlobalIndexAssignment::force_node_indicies() const {
    return _force_node_indicies;
}

int GlobalIndexAssignment::index(const QuadratureNode* node) const {
    return _quadrature_node_indicies.at(node);
}
int GlobalIndexAssignment::index(const ForceNode* node) const {
    return _force_node_indicies.at(node);
}

int GlobalIndexAssignment::index(const QuadratureNode& node) const {
    return _quadrature_node_indicies.at(&node);
}
int GlobalIndexAssignment::index(const ForceNode& node) const {
    return _force_node_indicies.at(&node);
}
}  // namespace cnearest