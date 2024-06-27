#include "discretisations/straight_line_discretisation.h"
#include "discretisations/single_point_discretisation.h"
#include "gtest/gtest.h"
#include "nearest_neighbour/nearest_neighbour_detection.h"
#include "nearest_neighbour/nearest_neighbour_distributers.h"

class NearestNeighbourSingleBodyTest : public ::testing::Test {
   protected:
    virtual void SetUp() {};

    virtual void TearDown() {};
};

TEST_F(NearestNeighbourSingleBodyTest, test_single_matching_point_detection) {
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back (discretisation, discretisation);

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

TEST_F(NearestNeighbourSingleBodyTest, test_single_force_node_detection) {
    cnearest::StraightLineDiscretisation line_discretisation({0, 0, 1}, {0, 0, -1}, 5);
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(discretisation, line_discretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 1);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 7);

    const cnearest::ForceNode& sole_force_node = bodies[0].force_nodes().nodes()[0];

    cnearest::NearestNeighbourDetectionAlgorithm algorithm;

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 7);

    for (auto& quadrature_node : bodies[0].quadrature_nodes().nodes()) {
        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();
        ASSERT_EQ(mapped_force_node, &sole_force_node);
    }
}

TEST_F(NearestNeighbourSingleBodyTest, test_two_slighly_offset_lines_detection) {
    cnearest::StraightLineDiscretisation quad_disretisation({0, 0, 1}, {0, 0, -1}, 15);
    cnearest::StraightLineDiscretisation force_disretisation({0.1, 0, 1}, {0.1, 0, -1}, 15);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back (force_disretisation, quad_disretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 17);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 17);

    cnearest::NearestNeighbourDetectionAlgorithm algorithm;

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 17);

    for (auto& quadrature_node : bodies[0].quadrature_nodes().nodes()) {
        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        auto offset = (mapped_force_node->position() - quadrature_node.position());

        ASSERT_EQ(offset(0), 0.1);
        ASSERT_EQ(offset(1), 0.0);
        ASSERT_EQ(offset(2), 0.0);
    }

    for (int i = 0; i < 17; i++) {
        const cnearest::ForceNode& force_node = bodies[0].force_nodes().nodes()[i];
        const cnearest::QuadratureNode& quadrature_node =
            bodies[0].quadrature_nodes().nodes()[i];

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_EQ(mapped_force_node, &force_node);
    }
}

TEST_F(NearestNeighbourSingleBodyTest, test_two_slighly_offset_lines_reversed_detection) {
    cnearest::StraightLineDiscretisation quad_disretisation({0, 0, 1}, {0, 0, -1}, 15);
    cnearest::StraightLineDiscretisation force_disretisation({0.1, 0, -1}, {0.1, 0, 1}, 15);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(force_disretisation, quad_disretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 17);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 17);

    cnearest::NearestNeighbourDetectionAlgorithm algorithm;

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 17);

    for (auto& quadrature_node : bodies[0].quadrature_nodes().nodes()) {
        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        auto offset = (mapped_force_node->position() - quadrature_node.position());

        ASSERT_EQ(offset(0), 0.1);
        ASSERT_EQ(offset(1), 0.0);
        ASSERT_EQ(offset(2), 0.0);
    }

    for (int i = 0; i < 17; i++) {
        const cnearest::ForceNode& force_node = bodies[0].force_nodes().nodes()[i];
        const cnearest::QuadratureNode& quadrature_node =
            bodies[0].quadrature_nodes().nodes()[16 - i];

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_EQ(mapped_force_node, &force_node);
    }
}

TEST_F(NearestNeighbourSingleBodyTest, test_ties_cause_error) {
    cnearest::StraightLineDiscretisation quad_disretisation({0, 0, -0.5}, {0, 0, 0.5});
    cnearest::StraightLineDiscretisation force_disretisation({0, 0, -1}, {0, 0, 1}, 1);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(force_disretisation, quad_disretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 3);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 2);

    cnearest::NearestNeighbourDetectionAlgorithm algorithm;
    ASSERT_THROW(auto nearest_neighbours = algorithm.nearest_neighbours(bodies);
                 , std::runtime_error);
}

TEST_F(NearestNeighbourSingleBodyTest, test_tiebreaking_with_first) {
    cnearest::StraightLineDiscretisation quad_disretisation({0, 0, -0.5}, {0, 0, 0.5});
    cnearest::StraightLineDiscretisation force_disretisation({0, 0, -1}, {0, 0, 1}, 1);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(force_disretisation, quad_disretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 3);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 2);

    auto distributer = std::make_unique<cnearest::NearestNeighbourDistributerFirst>();
    cnearest::NearestNeighbourDetectionAlgorithm algorithm(std::move(distributer));

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 2);

    {
        auto& quadrature_node = bodies[0].quadrature_nodes().nodes()[0];

        ASSERT_EQ(quadrature_node.position(), cnearest::Vector3({0, 0, -0.5}));

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_EQ(mapped_force_node->position(), cnearest::Vector3({0, 0, -1.0}));
    }

    {
        auto& quadrature_node = bodies[0].quadrature_nodes().nodes()[1];

        ASSERT_EQ(quadrature_node.position(), cnearest::Vector3({0, 0, 0.5}));

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_EQ(mapped_force_node->position(), cnearest::Vector3({0, 0, 0.0}));
    }
}

TEST_F(NearestNeighbourSingleBodyTest, test_tiebreaking_with_last) {
    cnearest::StraightLineDiscretisation quad_disretisation({0, 0, -0.5}, {0, 0, 0.5});
    cnearest::StraightLineDiscretisation force_disretisation({0, 0, -1}, {0, 0, 1}, 1);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(force_disretisation, quad_disretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 3);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 2);

    auto distributer = std::make_unique<cnearest::NearestNeighbourDistributerLast>();
    cnearest::NearestNeighbourDetectionAlgorithm algorithm(std::move(distributer));

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 2);

    {
        auto& quadrature_node = bodies[0].quadrature_nodes().nodes()[0];

        ASSERT_EQ(quadrature_node.position(), cnearest::Vector3({0, 0, -0.5}));

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_EQ(mapped_force_node->position(), cnearest::Vector3({0, 0, 0}));
    }

    {
        auto& quadrature_node = bodies[0].quadrature_nodes().nodes()[1];

        ASSERT_EQ(quadrature_node.position(), cnearest::Vector3({0, 0, 0.5}));

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_EQ(mapped_force_node->position(), cnearest::Vector3({0, 0, 1}));
    }
}

TEST_F(NearestNeighbourSingleBodyTest, test_tie_detection_with_tolerance) {
    cnearest::StraightLineDiscretisation quad_disretisation({0, 0, -0.5}, {0, 0, 0.5});
    cnearest::StraightLineDiscretisation force_disretisation({0, 0, -1}, {0, 0, 1}, 16);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(force_disretisation, quad_disretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 18);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 2);

    cnearest::NearestNeighbourDetectionAlgorithm algorithm;
    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 2);

    {
        auto& quadrature_node = bodies[0].quadrature_nodes().nodes()[0];

        ASSERT_EQ(quadrature_node.position(), cnearest::Vector3({0, 0, -0.5}));

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_DOUBLE_EQ(mapped_force_node->position()(2), -9.0 / 17.0);
    }

    {
        auto& quadrature_node = bodies[0].quadrature_nodes().nodes()[1];

        ASSERT_EQ(quadrature_node.position(), cnearest::Vector3({0, 0, 0.5}));

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_DOUBLE_EQ(mapped_force_node->position()(2), 9.0 / 17.0);
    }

    cnearest::NearestNeighbourDetectionAlgorithm algorithm_with_tolerance(nullptr, 0.06);
    ASSERT_THROW(
        auto nearest_neighbours = algorithm_with_tolerance.nearest_neighbours(bodies);
        , std::exception);
}

TEST_F(NearestNeighbourSingleBodyTest,
       test_tie_detection_with_tolerance_first_distributer) {
    cnearest::StraightLineDiscretisation quad_disretisation({0, 0, -0.5}, {0, 0, 0.5});
    cnearest::StraightLineDiscretisation force_disretisation({0, 0, -1}, {0, 0, 1}, 16);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(force_disretisation, quad_disretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 18);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 2);

    auto distributer = std::make_unique<cnearest::NearestNeighbourDistributerFirst>();
    cnearest::NearestNeighbourDetectionAlgorithm algorithm(std::move(distributer), 0.06);

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 2);

    {
        auto& quadrature_node = bodies[0].quadrature_nodes().nodes()[0];

        ASSERT_EQ(quadrature_node.position(), cnearest::Vector3({0, 0, -0.5}));

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_DOUBLE_EQ(mapped_force_node->position()(2), -9.0 / 17.0);
    }

    {
        auto& quadrature_node = bodies[0].quadrature_nodes().nodes()[1];

        ASSERT_EQ(quadrature_node.position(), cnearest::Vector3({0, 0, 0.5}));

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_DOUBLE_EQ(mapped_force_node->position()(2), 7.0 / 17.0);
    }
}

TEST_F(NearestNeighbourSingleBodyTest,
       test_tie_detection_with_tolerance_last_distributer) {
    cnearest::StraightLineDiscretisation quad_disretisation({0, 0, -0.5}, {0, 0, 0.5});
    cnearest::StraightLineDiscretisation force_disretisation({0, 0, -1}, {0, 0, 1}, 16);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(force_disretisation, quad_disretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 18);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 2);

    auto distributer = std::make_unique<cnearest::NearestNeighbourDistributerLast>();
    cnearest::NearestNeighbourDetectionAlgorithm algorithm(std::move(distributer), 0.06);

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 2);

    {
        auto& quadrature_node = bodies[0].quadrature_nodes().nodes()[0];

        ASSERT_EQ(quadrature_node.position(), cnearest::Vector3({0, 0, -0.5}));

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_DOUBLE_EQ(mapped_force_node->position()(2), -7.0 / 17.0);
    }

    {
        auto& quadrature_node = bodies[0].quadrature_nodes().nodes()[1];

        ASSERT_EQ(quadrature_node.position(), cnearest::Vector3({0, 0, 0.5}));

        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_DOUBLE_EQ(mapped_force_node->position()(2), 9.0 / 17.0);
    }
}

class EmptyDiscretisation : public cnearest::iDiscretisation {
   public:
    cnearest::Collection<cnearest::Vector3> positions() const override { return {}; }
};

TEST_F(NearestNeighbourSingleBodyTest, test_no_node_causes_error) {
    cnearest::SinglePointDiscretisation quad_disretisation(1, 2, 3);
    EmptyDiscretisation force_disretisation;

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(force_disretisation, quad_disretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 0);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 1);

    cnearest::NearestNeighbourDetectionAlgorithm algorithm;
    ASSERT_THROW(auto nearest_neighbours = algorithm.nearest_neighbours(bodies);
                 , std::runtime_error);
}
