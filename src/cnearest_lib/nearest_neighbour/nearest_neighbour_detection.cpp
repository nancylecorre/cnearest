#include "nearest_neighbour_detection.h"

namespace cnearest {

NearestNeighbourDetectionAlgorithm::NearestNeighbourDetectionAlgorithm(
    std::unique_ptr<iNearestNeighbourTiebreaker> tiebreaker, double tie_shell_thickness)
    : _tiebreaker(std::move(tiebreaker)), _tie_shell_thickness(tie_shell_thickness) {}

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
    Collection<const ForceNode*> potential_ties;
    double best_candidate_distance = std::numeric_limits<double>::infinity();

    for (auto& force_node : body.force_nodes().nodes()) {
        double d = (force_node.position() - quadrature_node.position()).norm();
        if (d < best_candidate_distance) {
            potential_ties.push_back(&force_node);
            best_candidate_distance = d;
            // this potenitally invalidates the existing list of
            // potential ties, so we will filter them again at the end.
        } else if (d <= best_candidate_distance + _tie_shell_thickness) {
            potential_ties.push_back(&force_node);
        }
    }

    // filter the potential ties list for invalidated ones (added before the best
    // candidate was found)
    potential_ties.erase(
        std::remove_if(
            potential_ties.begin(), potential_ties.end(),
            [&best_candidate_distance, &quadrature_node, this](const ForceNode* item) {
                double d = (item->position() - quadrature_node.position()).norm();
                return d > best_candidate_distance + _tie_shell_thickness;
            }),
        potential_ties.end());

    // return or invoke tiebreaker
    switch (potential_ties.size()) {
        case 1:
            results.insert(std::pair(&quadrature_node, NearestNeighbourResult(potential_ties[0])));
            break;
        case 0:
            throw std::exception("no valid neighbours");
            break;
        default:
            if (_tiebreaker)
                results.insert(std::pair(&quadrature_node,
                              NearestNeighbourResult(_tiebreaker->nearest_neighbour(
                                  quadrature_node, potential_ties))));
            else
                throw std::exception("There was a tie and no tiebreaker was provided");
            break;
    }
}
}  // namespace cnearest
