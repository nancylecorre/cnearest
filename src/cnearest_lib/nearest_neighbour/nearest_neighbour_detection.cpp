#include "nearest_neighbour_detection.h"

namespace cnearest {

    NearestNeighbourDetectionAlgorithm::NearestNeighbourDetectionAlgorithm(
        std::unique_ptr<iNearestNeighbourDistributer> distributer,
        double tie_shell_thickness)
        : _distributer(std::move(distributer)),
          _tie_shell_thickness(tie_shell_thickness) {}

    NearestNeighbourResults NearestNeighbourDetectionAlgorithm::nearest_neighbours(
        const Collection<RigidBody>& bodies) const {
        std::map<const QuadratureNode*, NearestNeighbourResult> results;

        for (auto& body : bodies) {
            for (auto& quadrature_node : body.quadrature_nodes().nodes()) {
                populate_nearest_neighbours(quadrature_node, body, results);
            }
        }
        return NearestNeighbourResults(results);
    }

    void NearestNeighbourDetectionAlgorithm::populate_nearest_neighbours(
        const QuadratureNode& quadrature_node, const RigidBody& body,
        std::map<const QuadratureNode*, NearestNeighbourResult>& results) const {
        double best_candidate_distance = std::numeric_limits<double>::infinity();

        //find the distance to the closest node
        for (auto& force_node : body.force_nodes().nodes()) {
            double d = (force_node.position() - quadrature_node.position()).norm();
            if (d < best_candidate_distance) {
                best_candidate_distance = d;
            } 
        }

        //find nodes at (or close to) the best distance
        Collection<const ForceNode*> tied_nodes;
        for (auto& force_node : body.force_nodes().nodes()) {
            if ((force_node.position() - quadrature_node.position()).norm() <=
                (best_candidate_distance + _tie_shell_thickness))
                tied_nodes.emplace_back(&force_node);
        }
        // return or invoke distributer
        switch (tied_nodes.size()) {
            case 1:
                results.insert({&quadrature_node, NearestNeighbourResult(tied_nodes[0])});
                break;
            case 0:
                throw std::runtime_error("no valid neighbours");
                break;
            default:
                if (_distributer)
                    results.insert({&quadrature_node,
                                    _distributer->nearest_neighbour_result(quadrature_node, tied_nodes)});
                else
                    throw std::runtime_error(
                        "There was a tie and no distributer was provided");
                break;
        }
    }
}
