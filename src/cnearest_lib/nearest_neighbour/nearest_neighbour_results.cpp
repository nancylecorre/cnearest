#include "nearest_neighbour_results.h"
#include <numeric>

namespace cnearest {
    NearestNeighbourResult::NearestNeighbourResult(
        const Collection<ForceNodeWithWeight>& result)
        : _result(result) {
    }

    NearestNeighbourResult::NearestNeighbourResult(const ForceNode* node)
        : _result({ForceNodeWithWeight{1.0, node}}) {}

    NearestNeighbourResults::NearestNeighbourResults(
        const std::map<const QuadratureNode*, NearestNeighbourResult>&
            nearest_neighbour_mapping)
        : _nearest_neighbour_mapping(nearest_neighbour_mapping) {

        //populate the map keys
        for (auto& [quadrature_node, r] : _nearest_neighbour_mapping) {
            for (auto& [weight, force_node] : r) {
                _quadrature_node_sum[force_node] = 0.0;
                _quadrature_node_average_position[force_node] = Vector3::Zero();
            }
        }

        //sum the positions and weights
        for (auto& [quadrature_node, r] : _nearest_neighbour_mapping) {
            for (auto& [weight, force_node] : r) {
                _quadrature_node_sum[force_node] += weight;
                _quadrature_node_average_position[force_node] += weight * quadrature_node->position();
            }
        }
        //convert to the mean of the positions
        for (auto& [force_node, _] : _quadrature_node_average_position)
            _quadrature_node_average_position[force_node] /= _quadrature_node_sum[force_node];
    }

    const std::map<const QuadratureNode*, NearestNeighbourResult>&
    NearestNeighbourResults::nearest_neighbour_mapping() const {
        return _nearest_neighbour_mapping;
    }

    const ForceNode* NearestNeighbourResult::sole() const {
        return _result.size() == 1
                   ? _result[0].force_node
                   : throw std::runtime_error("There was more than one element");
    }

    
    double NearestNeighbourResults::quadrature_node_sum(const ForceNode& node) const {
        return _quadrature_node_sum.at(&node);
    }

    Vector3 NearestNeighbourResults::quadrature_node_average_position(const ForceNode& node) const {
        return _quadrature_node_average_position.at(&node);
    }

}
