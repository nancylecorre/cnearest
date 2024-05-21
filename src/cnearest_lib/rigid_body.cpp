#include "rigid_body.h"

namespace cnearest {

    RigidBody::RigidBody(const iDiscretisation& force_discretisation,
                         const iDiscretisation& quadrature_discretisation)
        : _force_nodes({}), _quadrature_nodes({}) {
        auto force_positions = force_discretisation.positions();
        Collection<ForceNode> force_nodes;
        force_nodes.reserve(force_positions.size());
        std::transform(force_positions.begin(), force_positions.end(),
                       std::back_inserter(force_nodes),
                       [](const Vector3& v) { return ForceNode(v); });
        _force_nodes = ForceNodeSet(force_nodes);

        auto quadrature_positions = quadrature_discretisation.positions();
        Collection<QuadratureNode> quadrature_nodes;
        quadrature_nodes.reserve(quadrature_positions.size());
        std::transform(quadrature_positions.begin(), quadrature_positions.end(),
                       std::back_inserter(quadrature_nodes),
                       [](const Vector3& v) { return QuadratureNode(v); });
        _quadrature_nodes = QuadratureNodeSet(quadrature_nodes);
    }
    const ForceNodeSet& RigidBody::force_nodes() const { return _force_nodes; }

    const QuadratureNodeSet& RigidBody::quadrature_nodes() const {
        return _quadrature_nodes;
    }

}
