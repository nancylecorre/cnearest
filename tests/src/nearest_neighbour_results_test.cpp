#include "nearest_neighbour/nearest_neighbour_results.h"

#include "gtest/gtest.h"

TEST(TestNearestNeighbourResults, TestNearestNeighbourResultWithSoleNode) {
    cnearest::ForceNode node_1({0, 0, 0});

    cnearest::NearestNeighbourResult r(&node_1);

    ASSERT_EQ(r.sole(), &node_1);
}

TEST(TestNearestNeighbourResults, TestNearestNeighbourResultWithWeightedNodes) {
    cnearest::ForceNode node_1({0, 0, 0});
    cnearest::ForceNode node_2({0, 0, 0});

    cnearest::Collection<cnearest::NearestNeighbourResult::ForceNodeWithWeight> weights(
        2);
    weights[0] = {0.5, &node_1};
    weights[1] = {0.5, &node_2};

    cnearest::NearestNeighbourResult r(weights);
    for (auto& force_node_with_weight : r) {
        ASSERT_EQ(force_node_with_weight.weight, 0.5);
    }
}

TEST(TestNearestNeighbourResults, TestNearestNeighboutResults) {
    cnearest::QuadratureNode quadrature_node({0, 0, 0});
    cnearest::ForceNode node_1({0, 0, 0});
    cnearest::ForceNode node_2({0, 0, 0});

    cnearest::Collection<cnearest::NearestNeighbourResult::ForceNodeWithWeight> weights(
        2);
    weights[0] = {0.5, &node_1};
    weights[1] = {0.5, &node_2};

    cnearest::NearestNeighbourResult r(weights);
    cnearest::NearestNeighbourResults rs({{&quadrature_node, r}});

    for (auto& res : rs.nearest_neighbour_mapping()) {
        ASSERT_EQ(res.first, &quadrature_node);
        for (auto& force_node_with_weight : res.second) {
            ASSERT_EQ(force_node_with_weight.weight, 0.5);
        }
    }
}

TEST(TestNearestNeighbourResults, TestNearestNeighboutResultsStructuredBinding) {
    cnearest::QuadratureNode quadrature_node({0, 0, 0});
    cnearest::ForceNode node_1({0, 0, 0});
    cnearest::ForceNode node_2({0, 0, 0});

    cnearest::Collection<cnearest::NearestNeighbourResult::ForceNodeWithWeight> weights(
        2);
    weights[0] = {0.5, &node_1};
    weights[1] = {0.5, &node_2};

    cnearest::NearestNeighbourResult r(weights);
    cnearest::NearestNeighbourResults rs({{&quadrature_node, r}});

    for (auto const& [node, result] : rs.nearest_neighbour_mapping()) {
        ASSERT_EQ(node, &quadrature_node);
        for (auto& force_node_with_weight : result) {
            ASSERT_EQ(force_node_with_weight.weight, 0.5);
        }
    }
}
