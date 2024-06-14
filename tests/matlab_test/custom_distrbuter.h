
#include "nearest_neighbour/nearest_neighbour_distributers.h"
#include "global_index_assignment.h"
#include"nearest_neighbour/nearest_neighbour_detection.h"


class NearestNeighbourDistributerForMatlabTest : public cnearest::iNearestNeighbourDistributer {
   public:
    NearestNeighbourDistributerForMatlabTest(const cnearest::GlobalIndexAssignment& a) : assignment(a) {}

    cnearest::NearestNeighbourResult nearest_neighbour_result(
        const cnearest::QuadratureNode& n,
        cnearest::Collection<const cnearest::ForceNode*> candidates) const override {
        for (auto& [a, b] : mapping) {
            if (assignment.index(n) == a) {
                auto findResult = std::find_if(std::begin(assignment.force_node_indicies()),
                                               std::end(assignment.force_node_indicies()),
                                               [&](auto& a) { return a.second == b; });
                return findResult->first;
            }
        }
        throw "unmapped tied node";
    }

    const cnearest::GlobalIndexAssignment& assignment;
    // overrule some of the detected nearest neighbours with ones exactly matching the matlab ones
    std::map<int, int> mapping = {{234, 18}, {239, 11}, {264, 30}, {269, 11},
                                  {594, 18}, {599, 47}, {624, 30}, {629, 47}};
};

// We need a very special detection algorithm to make sure that the 8 near-tied nodes get
// mapped to the same ones as matlab, despite their different numerical errors
cnearest::NearestNeighbourDetectionAlgorithm matlab_matching_detection(
    const cnearest::GlobalIndexAssignment& a) {
    // define our matlab inspired tiebreaker
    auto tiebreaker = std::make_unique<NearestNeighbourDistributerForMatlabTest>(a);
    // Detect the nearest neighbours
    return cnearest::NearestNeighbourDetectionAlgorithm(std::move(tiebreaker), 1.0e-9);
}
