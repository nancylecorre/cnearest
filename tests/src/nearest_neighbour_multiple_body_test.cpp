#include "gtest/gtest.h"
#include "nearest_neighbour/nearest_neighbour_tiebreakers.h"
#include "nearest_neighbour/nearest_neighbour_detection.h"
#include "discretisations/single_point_discretisation.h"
#include "discretisations/line_discretisation.h"

class NearestNeighbourMultipleBodyTest : public ::testing::Test {
   protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(NearestNeighbourMultipleBodyTest, test_multiple_body_detection) {
    cnearest::SinglePointDiscretisation point_discretisation_1(0,0,1);
    cnearest::SinglePointDiscretisation point_discretisation_2(0,0,2);
    cnearest::SinglePointDiscretisation point_discretisation_3(0,0,3);
    cnearest::SinglePointDiscretisation point_discretisation_4(0,0,4);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.push_back(cnearest::RigidBody(point_discretisation_1, point_discretisation_2));
    bodies.push_back(cnearest::RigidBody(point_discretisation_3, point_discretisation_4));

    ASSERT_EQ(bodies.size(), 2);
    ASSERT_EQ(bodies[0].force_nodes().size(), 1);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 1);
    ASSERT_EQ(bodies[1].force_nodes().size(), 1);
    ASSERT_EQ(bodies[1].quadrature_nodes().size(), 1);

    cnearest::NearestNeighbourDetectionAlgorithm algorithm;

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 2);

    for (auto& quadrature_node : bodies[0].quadrature_nodes().nodes()) {
        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_EQ(mapped_force_node->position()(0), 0);
        ASSERT_EQ(mapped_force_node->position()(1), 0);
        ASSERT_EQ(mapped_force_node->position()(2), 1);
    }

    for (auto& quadrature_node : bodies[1].quadrature_nodes().nodes()) {
        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_EQ(mapped_force_node->position()(0), 0);
        ASSERT_EQ(mapped_force_node->position()(1), 0);
        ASSERT_EQ(mapped_force_node->position()(2), 3);
    }
}


TEST_F(NearestNeighbourMultipleBodyTest, test_no_inter_body_detection) {
    cnearest::LineDiscretisation line_disretisation({0, 0, 1}, {0, 0, -1}, 5);
    cnearest::SinglePointDiscretisation point_discretisation(1, 1, 0.6);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.push_back(cnearest::RigidBody(point_discretisation, line_disretisation));
    bodies.push_back(cnearest::RigidBody(line_disretisation, point_discretisation));

    ASSERT_EQ(bodies.size(), 2);
    ASSERT_EQ(bodies[0].force_nodes().size(), 1);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 7);
    ASSERT_EQ(bodies[1].force_nodes().size(), 7);
    ASSERT_EQ(bodies[1].quadrature_nodes().size(), 1);
    
    cnearest::NearestNeighbourDetectionAlgorithm algorithm;

    auto nearest_neighbours = algorithm.nearest_neighbours(bodies);

    ASSERT_EQ(nearest_neighbours.nearest_neighbour_mapping().size(), 7+1);

    //body 0, all 7 quadrature nodes should get mapped to the single  
    for (auto& quadrature_node : bodies[0].quadrature_nodes().nodes()) {
        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_EQ(mapped_force_node->position()(0), 1);
        ASSERT_EQ(mapped_force_node->position()(1), 1);
        ASSERT_EQ(mapped_force_node->position()(2), 0.6);
    }

    //body 1, the single quadrature node should be mapped to the nearest node on the line
    for (auto& quadrature_node : bodies[1].quadrature_nodes().nodes()) {
        auto mapped_force_node =
            nearest_neighbours.nearest_neighbour_mapping().at(&quadrature_node).sole();

        ASSERT_EQ(mapped_force_node->position()(0), 0);
        ASSERT_EQ(mapped_force_node->position()(1), 0);
        ASSERT_NEAR(mapped_force_node->position()(2), 2.0/3.0, 1.0e-12);
    }
}

