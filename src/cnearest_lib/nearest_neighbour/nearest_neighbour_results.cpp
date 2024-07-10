#include "nearest_neighbour_results.h"
#include <numeric>

namespace cnearest {
    NearestNeighbourResult::NearestNeighbourResult(
        const Collection<ForceNodeWithWeight>& result)
        : _result(result) {}

    NearestNeighbourResult::NearestNeighbourResult(const ForceNode* node)
        : _result({ForceNodeWithWeight{1.0, node}}) {}

    
    double NearestNeighbourResult::quadrature_node_sum(const ForceNode& node) const {
        auto lambda =
            [&node](double a, const ForceNodeWithWeight& b) {
            return a + (b.force_node == &node ? b.weight : 0.0);
            };
        return std::accumulate(_result.begin(), _result.end(), 0.0, lambda);
    }

    NearestNeighbourResults::NearestNeighbourResults(
        const std::map<const QuadratureNode*, NearestNeighbourResult>&
            nearest_neighbour_mapping)
        : _nearest_neighbour_mapping(nearest_neighbour_mapping) {}

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
        auto lambda =
            [&node](double a,
               const std::pair<const QuadratureNode*, NearestNeighbourResult>& b) {
                return a + b.second.quadrature_node_sum(node);
            };
        return std::accumulate(_nearest_neighbour_mapping.begin(),
                               _nearest_neighbour_mapping.end(), 
                                0.0, 
                                lambda);
    }

    Vector3 NearestNeighbourResults::quadrature_node_average_position(const ForceNode& node) const {
        Vector3 sum_positions = Vector3::Zero();
        for (auto& [quadrature_node, r] : _nearest_neighbour_mapping) {
            for (auto& [weight, force_node] : r) {
                if (force_node == &node) sum_positions += weight * quadrature_node->position();
            }
        }

        return sum_positions / quadrature_node_sum(node);
    }

}
