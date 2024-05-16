#include "gtest/gtest.h"
#include "nearest_neighbour/nearest_neighbour_tiebreakers.h"
#include "nearest_neighbour_detection.h"
#include "discretisations/single_point_discretisation.h"


class NearestNeighbourMultipleBodyParameterizedTests
    : public ::testing::TestWithParam<cnearest::Vector3> {};

TEST_P(NearestNeighbourMultipleBodyParameterizedTests, TestQuadraturePosition) {
    cnearest::Vector3 position = GetParam();
    cnearest::SinglePointDiscretisation discretisation(position(0), position(1),
                                                       position(2));
    cnearest::SinglePointDiscretisation force_discretisation(0, 0, 0);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.push_back(cnearest::RigidBody(force_discretisation, discretisation));

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 1);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 1);

    const cnearest::ForceNode& sole_force_node = bodies[0].force_nodes().nodes()[0];
    const cnearest::QuadratureNode& sole_quadrature_node =
        bodies[0].quadrature_nodes().nodes()[0];

    cnearest::NearestNeighbourDetectionAlgorithm algorithm;

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 1);
    auto sole_mapping =
        nearest_neighbours.nearest_neighbour_mapping().at(&sole_quadrature_node).sole();
    ASSERT_EQ(sole_mapping, &sole_force_node);
}

TEST_P(NearestNeighbourMultipleBodyParameterizedTests, TestForcePosition) {
    cnearest::Vector3 position = GetParam();
    cnearest::SinglePointDiscretisation discretisation(position(0), position(1),
                                                       position(2));
    cnearest::SinglePointDiscretisation quadrature_discretisation(0, 0, 0);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.push_back(cnearest::RigidBody(discretisation, quadrature_discretisation));

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 1);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 1);

    const cnearest::ForceNode& sole_force_node = bodies[0].force_nodes().nodes()[0];
    const cnearest::QuadratureNode& sole_quadrature_node =
        bodies[0].quadrature_nodes().nodes()[0];

    cnearest::NearestNeighbourDetectionAlgorithm algorithm;

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 1);
    auto sole_mapping =
        nearest_neighbours.nearest_neighbour_mapping().at(&sole_quadrature_node).sole();
    ASSERT_EQ(sole_mapping, &sole_force_node);
}


INSTANTIATE_TEST_SUITE_P(
    TestExtremeVaues, NearestNeighbourMultipleBodyParameterizedTests,
    ::testing::Values(cnearest::Vector3({0, 0, std::numeric_limits<double>::max()}),
                      cnearest::Vector3({0, 0, 1e8}), cnearest::Vector3({0, 0, 1000}),
                      cnearest::Vector3({0, 0, 1.0}),
                      cnearest::Vector3({0, 0, std::numeric_limits<double>::min()}),
                      cnearest::Vector3({0, 0, 0.0}),
                      cnearest::Vector3({0, 0, -std::numeric_limits<double>::min()}),
                      cnearest::Vector3({0, 0, -1.0}), cnearest::Vector3({0, 0, -1000}),
                      cnearest::Vector3({0, 0, -1e8}),
                      cnearest::Vector3({0, 0, -std::numeric_limits<double>::max()})));
