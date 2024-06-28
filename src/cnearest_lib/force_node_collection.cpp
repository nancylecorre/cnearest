#include "force_node_collection.h"

namespace cnearest {

    ForceNodeCollection::ForceNodeCollection(const Collection<ForceNode>& nodes) : _nodes(nodes) {}

    
    ForceNodeCollection::ForceNodeCollection(const Collection<Vector3>& positions) {
        _nodes.reserve(positions.size());
        std::transform(positions.begin(), positions.end(), std::back_inserter(_nodes),
                       [](const Vector3& v) { return ForceNode(v); });
    }

    ForceNodeCollection::~ForceNodeCollection() {}

    const Collection<ForceNode>& ForceNodeCollection::nodes() const { return _nodes; }

    const size_t ForceNodeCollection::size() const { return _nodes.size(); }

}
