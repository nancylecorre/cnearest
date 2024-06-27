#include "discretisations/straight_line_discretisation.h"
#include "discretisations/single_point_discretisation.h"
#include "gtest/gtest.h"
#include "global_index_assignment.h"

class GlobalIndexTest : public ::testing::Test {
   protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(GlobalIndexTest, test_single_body_single_node) {
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(discretisation, discretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 1);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 1);

    const cnearest::ForceNode& sole_force_node = bodies[0].force_nodes().nodes()[0];
    const cnearest::QuadratureNode& sole_quadrature_node =
        bodies[0].quadrature_nodes().nodes()[0];

    cnearest::GlobalIndexAssignment assignment(bodies);


    ASSERT_EQ(assignment.quadrature_node_indicies().size(), 1);
    ASSERT_EQ(assignment.quadrature_node_indicies().at(&sole_quadrature_node), 0);
    ASSERT_EQ(assignment.index(&sole_quadrature_node), 0);
    ASSERT_EQ(assignment.index(sole_quadrature_node), 0);

    ASSERT_EQ(assignment.force_node_indicies().size(), 1);
    ASSERT_EQ(assignment.force_node_indicies().at(&sole_force_node), 0);
    ASSERT_EQ(assignment.index(&sole_force_node), 0);
    ASSERT_EQ(assignment.index(sole_force_node), 0);
    
}


TEST_F(GlobalIndexTest, test_single_body_multiple_quadrature_node) {
    cnearest::StraightLineDiscretisation line_disretisation({0, 0, 1}, {0, 0, -1}, 5);
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(discretisation, line_disretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 1);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 7);

    const cnearest::ForceNode& sole_force_node = bodies[0].force_nodes().nodes()[0];

    cnearest::GlobalIndexAssignment assignment(bodies);

    ASSERT_EQ(assignment.quadrature_node_indicies().size(), 7);
    for (int i = 0; i < 7; i++) {
        ASSERT_EQ(assignment.quadrature_node_indicies().at(
                      &bodies[0].quadrature_nodes().nodes()[i]),
                  i);
        ASSERT_EQ(assignment.index(
                      bodies[0].quadrature_nodes().nodes()[i]),
                  i);
    }

    ASSERT_EQ(assignment.force_node_indicies().size(), 1);
    ASSERT_EQ(assignment.force_node_indicies().at(&sole_force_node), 0);
    ASSERT_EQ(assignment.index(&sole_force_node), 0);
}


TEST_F(GlobalIndexTest, test_single_body_multiple_force_node) {
    cnearest::StraightLineDiscretisation line_disretisation({0, 0, 1}, {0, 0, -1}, 5);
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(line_disretisation, discretisation);

    ASSERT_EQ(bodies.size(), 1);
    ASSERT_EQ(bodies[0].force_nodes().size(), 7);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 1);

    const cnearest::QuadratureNode& sole_quadrature_node =
        bodies[0].quadrature_nodes().nodes()[0];

    cnearest::GlobalIndexAssignment assignment(bodies);

    ASSERT_EQ(assignment.quadrature_node_indicies().size(), 1);
    ASSERT_EQ(assignment.quadrature_node_indicies().at(&sole_quadrature_node), 0);
    ASSERT_EQ(assignment.index(&sole_quadrature_node), 0);

    ASSERT_EQ(assignment.force_node_indicies().size(), 7);
    for (int i = 0; i < 7; i++) {
        ASSERT_EQ(
            assignment.force_node_indicies().at(&bodies[0].force_nodes().nodes()[i]), i);
        ASSERT_EQ(
            assignment.index(bodies[0].force_nodes().nodes()[i]), i);
    }
}


TEST_F(GlobalIndexTest, test_multiple_body_single_node) {
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(discretisation, discretisation);
    bodies.emplace_back(discretisation, discretisation);

    ASSERT_EQ(bodies.size(), 2);
    ASSERT_EQ(bodies[0].force_nodes().size(), 1);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 1);
    ASSERT_EQ(bodies[1].force_nodes().size(), 1);
    ASSERT_EQ(bodies[1].quadrature_nodes().size(), 1);


    cnearest::GlobalIndexAssignment assignment(bodies);

    ASSERT_EQ(assignment.quadrature_node_indicies().size(), 2);
    ASSERT_EQ(assignment.index(bodies[0].quadrature_nodes().nodes()[0]), 0);
    ASSERT_EQ(assignment.index(bodies[1].quadrature_nodes().nodes()[0]), 1);

    ASSERT_EQ(assignment.force_node_indicies().size(), 2);
    ASSERT_EQ(assignment.index(bodies[0].force_nodes().nodes()[0]), 0);
    ASSERT_EQ(assignment.index(bodies[1].force_nodes().nodes()[0]), 1);
}


TEST_F(GlobalIndexTest, test_multiple_body_multiple_node) {
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);
    cnearest::StraightLineDiscretisation line_disretisation({0, 0, 1}, {0, 0, -1}, 5);

    cnearest::Collection<cnearest::RigidBody> bodies;
    bodies.emplace_back(discretisation, line_disretisation);
    bodies.emplace_back(line_disretisation, discretisation);

    ASSERT_EQ(bodies.size(), 2);
    ASSERT_EQ(bodies[0].force_nodes().size(), 1);
    ASSERT_EQ(bodies[0].quadrature_nodes().size(), 7);
    ASSERT_EQ(bodies[1].force_nodes().size(), 7);
    ASSERT_EQ(bodies[1].quadrature_nodes().size(), 1);

    cnearest::GlobalIndexAssignment assignment(bodies);

    ASSERT_EQ(assignment.quadrature_node_indicies().size(), 8);
    ASSERT_EQ(assignment.index(bodies[0].quadrature_nodes().nodes()[0]), 0);
    ASSERT_EQ(assignment.index(bodies[0].quadrature_nodes().nodes()[6]), 6);
    ASSERT_EQ(assignment.index(bodies[1].quadrature_nodes().nodes()[0]), 7);

    ASSERT_EQ(assignment.force_node_indicies().size(), 8);
    ASSERT_EQ(assignment.index(bodies[0].force_nodes().nodes()[0]), 0);
    ASSERT_EQ(assignment.index(bodies[1].force_nodes().nodes()[0]), 1);
    ASSERT_EQ(assignment.index(bodies[1].force_nodes().nodes()[6]), 7);
}