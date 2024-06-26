#include "force_node_collection.h"

namespace cnearest {

    ForceNodeCollection::ForceNodeCollection(const Collection<ForceNode>& nodes) : _nodes(nodes) {}

    ForceNodeCollection::~ForceNodeCollection() {}

    const Collection<ForceNode>& ForceNodeCollection::nodes() const { return _nodes; }

    const size_t ForceNodeCollection::size() const { return _nodes.size(); }

}
