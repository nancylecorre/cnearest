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

    
TEST(TestNearestNeighbourResults, TestNearestNeighboutResultsQuadratureNodeSum) {
        cnearest::QuadratureNode quadrature_node_1({0, 0, 0});
        cnearest::QuadratureNode quadrature_node_2({0, 0, 0});
        cnearest::QuadratureNode quadrature_node_3({0, 0, 0});
        cnearest::ForceNode node_1({0, 0, 0});
        cnearest::ForceNode node_2({0, 0, 0});

        cnearest::Collection<cnearest::NearestNeighbourResult::ForceNodeWithWeight>
            weights_1(2);
        weights_1[0] = {0.5, &node_1};
        weights_1[1] = {0.5, &node_2};

        
        cnearest::Collection<cnearest::NearestNeighbourResult::ForceNodeWithWeight>
            weights_2(2);
        weights_2[0] = {0.75, &node_1};
        weights_2[1] = {0.25, &node_2};

        
        cnearest::Collection<cnearest::NearestNeighbourResult::ForceNodeWithWeight>
            weights_3(2);
        weights_3[0] = {0.0, &node_1};
        weights_3[1] = {1.0, &node_2};

        cnearest::NearestNeighbourResult r_1(weights_1);
        cnearest::NearestNeighbourResult r_2(weights_2);
        cnearest::NearestNeighbourResult r_3(weights_3);
        cnearest::NearestNeighbourResults rs({{&quadrature_node_1, r_1},
                                              {&quadrature_node_2, r_2},
                                              {&quadrature_node_3, r_3}});

        
        ASSERT_DOUBLE_EQ(rs.quadrature_node_sum(node_1), 1.25);
        ASSERT_DOUBLE_EQ(rs.quadrature_node_sum(node_2), 1.75);

}

 
TEST(TestNearestNeighbourResults, TestNearestNeighboutResultsQuadratureNodeSumWithWeirdDuplicate) {
    cnearest::QuadratureNode quadrature_node_1({0, 0, 0});
    cnearest::QuadratureNode quadrature_node_2({0, 0, 0});
    cnearest::QuadratureNode quadrature_node_3({0, 0, 0});
    cnearest::ForceNode node_1({0, 0, 0});
    cnearest::ForceNode node_2({0, 0, 0});

    cnearest::Collection<cnearest::NearestNeighbourResult::ForceNodeWithWeight> weights_1(
        2);
    weights_1[0] = {0.5, &node_1};
    weights_1[1] = {0.5, &node_2};


    //This is weird but I'm checking it works if node1 apears twice, just in case
    cnearest::Collection<cnearest::NearestNeighbourResult::ForceNodeWithWeight> weights_2(
        3);
    weights_2[0] = {0.25, &node_1};
    weights_2[1] = {0.5, &node_1};
    weights_2[2] = {0.25, &node_2};

    cnearest::Collection<cnearest::NearestNeighbourResult::ForceNodeWithWeight> weights_3(
        2);
    weights_3[0] = {0.0, &node_1};
    weights_3[1] = {1.0, &node_2};

    cnearest::NearestNeighbourResult r_1(weights_1);
    cnearest::NearestNeighbourResult r_2(weights_2);
    cnearest::NearestNeighbourResult r_3(weights_3);
    cnearest::NearestNeighbourResults rs({{&quadrature_node_1, r_1},
                                          {&quadrature_node_2, r_2},
                                          {&quadrature_node_3, r_3}});

    ASSERT_DOUBLE_EQ(rs.quadrature_node_sum(node_1), 1.25);
    ASSERT_DOUBLE_EQ(rs.quadrature_node_sum(node_2), 1.75);
}