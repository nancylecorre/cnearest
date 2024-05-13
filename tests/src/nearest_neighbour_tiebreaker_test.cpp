#include "nearest_neighbout_tiebreakers.h"
#include "gtest/gtest.h"

TEST(TestNearestNeighbourTiebreakers, TestNearestNeighboutTiebreakerFirst) {

    cnearest::QuadratureNode quadrature_node({0,0,0});
    cnearest::ForceNode node_1({0, 0, 0});
    cnearest::ForceNode node_2({0, 0, 0});

    cnearest::Collection<const cnearest::ForceNode*> candidates = {&node_1, &node_2};

    cnearest::NearestNeighboutTiebreakerFirst tiebreaker;

    auto result = tiebreaker.nearest_neighbour(quadrature_node, candidates);

    ASSERT_EQ(result, &node_1);
}

TEST(TestNearestNeighbourTiebreakers, TestNearestNeighboutTiebreakerLast) {

    cnearest::QuadratureNode quadrature_node({0, 0, 0});
    cnearest::ForceNode node_1({0, 0, 0});
    cnearest::ForceNode node_2({0, 0, 0});

    cnearest::Collection<const cnearest::ForceNode*> candidates = {&node_1, &node_2};

    cnearest::NearestNeighboutTiebreakerLast tiebreaker;

    auto result = tiebreaker.nearest_neighbour(quadrature_node, candidates);

    ASSERT_EQ(result, &node_2);
}