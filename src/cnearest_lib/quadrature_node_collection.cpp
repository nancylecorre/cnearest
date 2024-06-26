#include "quadrature_node_collection.h"

namespace cnearest {

    QuadratureNodeCollection::QuadratureNodeCollection(const Collection<QuadratureNode>& nodes)
        : _nodes(nodes) {}

    QuadratureNodeCollection::~QuadratureNodeCollection() {}

    const Collection<QuadratureNode>& QuadratureNodeCollection::nodes() const { return _nodes; }

    const size_t QuadratureNodeCollection::size() const { return _nodes.size(); }

}
