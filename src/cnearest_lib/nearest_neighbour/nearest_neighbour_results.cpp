#include "nearest_neighbour_results.h"

namespace cnearest {
    NearestNeighbourResult::NearestNeighbourResult(
        const Collection<ForceNodeWithWeight>& result)
        : _result(result) {}

    NearestNeighbourResult::NearestNeighbourResult(const ForceNode* node)
        : _result({ForceNodeWithWeight{1.0, node}}) {}

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
}
