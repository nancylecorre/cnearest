#include "force_node_set.h"

namespace cnearest {

    ForceNodeSet::ForceNodeSet(const Collection<ForceNode>& nodes) : _nodes(nodes) {}

    ForceNodeSet::~ForceNodeSet() {}

    const Collection<ForceNode>& ForceNodeSet::nodes() const { return _nodes; }

    const size_t ForceNodeSet::size() const { return _nodes.size(); }

}
