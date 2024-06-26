#include "quadrature_node_collection.h"

namespace cnearest {

    QuadratureNodeCollection::QuadratureNodeCollection(const Collection<QuadratureNode>& nodes)
        : _nodes(nodes) {}

    QuadratureNodeCollection::QuadratureNodeCollection(const Collection<Vector3>& positions) {
        _nodes.reserve(positions.size());
        std::transform(positions.begin(), positions.end(), std::back_inserter(_nodes),
                       [](const Vector3& v) { return QuadratureNode(v); });
    }

    QuadratureNodeCollection::~QuadratureNodeCollection() {}

    const Collection<QuadratureNode>& QuadratureNodeCollection::nodes() const { return _nodes; }

    const size_t QuadratureNodeCollection::size() const { return _nodes.size(); }

}
