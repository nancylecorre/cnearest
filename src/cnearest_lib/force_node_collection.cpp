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

    
    double ForceNodeCollection::force_discretisation_spacing() const {
        double result_sqd = 0;
        for (auto& node : _nodes) {
            double node_minimal_distance_sqd = std::numeric_limits<double>::max();
            for (auto& inner_node : _nodes) {
                double offset_x = node.position().data()[0] - inner_node.position().data()[0];
                double offset_y = node.position().data()[1] - inner_node.position().data()[1];
                double offset_z = node.position().data()[2] - inner_node.position().data()[2];
                double d2 = offset_x * offset_x + offset_y * offset_y + offset_z * offset_z;
                if (&node != &inner_node && d2 < node_minimal_distance_sqd) node_minimal_distance_sqd = d2;
            }
            if (node_minimal_distance_sqd > result_sqd) result_sqd = node_minimal_distance_sqd;
        }
        return sqrt(result_sqd);
    }
}
