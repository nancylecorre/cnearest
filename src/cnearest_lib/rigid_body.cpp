#include "rigid_body.h"

namespace cnearest {

    RigidBody::RigidBody(const iDiscretisation& force_discretisation,
                         const iDiscretisation& quadrature_discretisation)
        : _force_nodes(force_discretisation.positions()),
          _quadrature_nodes(quadrature_discretisation.positions()),
          _origin_point({0,0,0}) {
    }

    RigidBody::RigidBody(const iDiscretisation& force_discretisation,
                         const iDiscretisation& quadrature_discretisation,
                         const Vector3& displacement, const Matrix3& rotation)
        : _force_nodes(Collection<ForceNode>{}),
          _quadrature_nodes(Collection<QuadratureNode>{}), 
          _origin_point(displacement) {
        auto force_positions = force_discretisation.positions();
        Collection<ForceNode> force_nodes;
        force_nodes.reserve(force_positions.size());
        std::transform(force_positions.begin(), force_positions.end(),
                       std::back_inserter(force_nodes), [&rotation, &displacement](const Vector3& v) {
                           return ForceNode(rotation * v + displacement);
                       });
        _force_nodes = ForceNodeCollection(force_nodes);

        auto quadrature_positions = quadrature_discretisation.positions();
        Collection<QuadratureNode> quadrature_nodes;
        quadrature_nodes.reserve(quadrature_positions.size());
        std::transform(quadrature_positions.begin(), quadrature_positions.end(),
                       std::back_inserter(quadrature_nodes),
                       [&rotation, &displacement](const Vector3& v) {
                           return QuadratureNode(rotation * v + displacement);
                       });
        _quadrature_nodes = QuadratureNodeCollection(quadrature_nodes);
    }

    const ForceNodeCollection& RigidBody::force_nodes() const { return _force_nodes; }

    const QuadratureNodeCollection& RigidBody::quadrature_nodes() const {
        return _quadrature_nodes;
    }

    const Vector3& RigidBody::origin_point() const { return _origin_point; }
}
