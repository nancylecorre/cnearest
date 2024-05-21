#include "quadrature_node_set.h"

namespace cnearest {

    QuadratureNodeSet::QuadratureNodeSet(const Collection<QuadratureNode>& nodes)
        : _nodes(nodes) {}

    QuadratureNodeSet::~QuadratureNodeSet() {}

    const Collection<QuadratureNode>& QuadratureNodeSet::nodes() const { return _nodes; }

const size_t QuadratureNodeSet::size() const { return _nodes.size(); }

}