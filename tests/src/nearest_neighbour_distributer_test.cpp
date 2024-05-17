#include "nearest_neighbour/nearest_neighbour_distributers.h"
#include "gtest/gtest.h"

TEST(TestNearestNeighbourDistributers, TestNearestNeighboutDistributerFirst) {

    cnearest::QuadratureNode quadrature_node({0,0,0});
    cnearest::ForceNode node_1({0, 0, 0});
    cnearest::ForceNode node_2({0, 0, 0});

    cnearest::Collection<const cnearest::ForceNode*> candidates = {&node_1, &node_2};

    cnearest::NearestNeighbourDistributerFirst distributer;

    auto result = distributer.nearest_neighbour_result(quadrature_node, candidates);

    ASSERT_EQ(result.sole(), &node_1);
}

TEST(TestNearestNeighbourDistributers, TestNearestNeighboutDistributerLast) {

    cnearest::QuadratureNode quadrature_node({0, 0, 0});
    cnearest::ForceNode node_1({0, 0, 0});
    cnearest::ForceNode node_2({0, 0, 0});

    cnearest::Collection<const cnearest::ForceNode*> candidates = {&node_1, &node_2};

    cnearest::NearestNeighbourDistributerLast distributer;

    auto result = distributer.nearest_neighbour_result(quadrature_node, candidates);

    ASSERT_EQ(result.sole(), &node_2);
}