#include "matrix_assembly/matrix_assembler_base.h"

#include <optional>

#include "discretisations/line_discretisation.h"
#include "global_index_assignment.h"
#include "gtest/gtest.h"

class MatrixAssemblerBaseTest : public ::testing::Test {
   protected:
    cnearest::Collection<cnearest::RigidBody> bodies;
    std::optional<cnearest::NearestNeighbourResults> nearest_neighbours;
    std::optional<cnearest::GlobalIndexAssignment> assignment;
    virtual void SetUp() {
        cnearest::LineDiscretisation force_discretisation({0, 0, 1}, {0, 0, -1}, 3);
        cnearest::LineDiscretisation quadrature_discretisation({0, 0, 1.05}, {0, 0, -1.05}, 10);

        bodies.push_back(cnearest::RigidBody(force_discretisation, quadrature_discretisation));

        assignment = cnearest::GlobalIndexAssignment(bodies);
        cnearest::NearestNeighbourDetectionAlgorithm algorithm;
        nearest_neighbours = algorithm.nearest_neighbours(bodies);
    };

    virtual void TearDown() {};
};

TEST_F(MatrixAssemblerBaseTest, test_m) {
    cnearest::MatrixAssemblerBase assembler(*assignment, *nearest_neighbours);
    ASSERT_EQ(assembler.number_of_force_nodes(), 5);
}

TEST_F(MatrixAssemblerBaseTest, test_q) {
    cnearest::MatrixAssemblerBase assembler(*assignment, *nearest_neighbours);
    ASSERT_EQ(assembler.number_of_quadrature_nodes(), 12);
}

TEST_F(MatrixAssemblerBaseTest, test_force_positions) {
    cnearest::MatrixAssemblerBase assembler(*assignment, *nearest_neighbours);
    auto v = assembler.concatenated_vector_of_force_positions();
    ASSERT_EQ(v.size(), 3 * 5);
    for (auto& [node, index] : assignment->force_node_indicies()) {
        ASSERT_DOUBLE_EQ(node->position().x(), v[3 * index + 0]);
        ASSERT_DOUBLE_EQ(node->position().y(), v[3 * index + 1]);
        ASSERT_DOUBLE_EQ(node->position().z(), v[3 * index + 2]);
    }
}

TEST_F(MatrixAssemblerBaseTest, test_quadrature_positions) {
    cnearest::MatrixAssemblerBase assembler(*assignment, *nearest_neighbours);
    auto v = assembler.concatenated_vector_of_quadrature_positions();
    ASSERT_EQ(v.size(), 3 * 12);
    for (auto& [node, index] : assignment->quadrature_node_indicies()) {
        ASSERT_DOUBLE_EQ(node->position().x(), v[3 * index + 0]);
        ASSERT_DOUBLE_EQ(node->position().y(), v[3 * index + 1]);
        ASSERT_DOUBLE_EQ(node->position().z(), v[3 * index + 2]);
    }
}

TEST_F(MatrixAssemblerBaseTest, test_neighbour_matrix) {
    cnearest::MatrixAssemblerBase assembler(*assignment, *nearest_neighbours);
    auto v = assembler.nearest_neighbour_matrix();
    ASSERT_EQ(v.rows(), 12);
    ASSERT_EQ(v.cols(), 5);
    for (int i = 0; i < 12; i++) {
        ASSERT_EQ(v.row(i).sum(), 1.0) << "Each quadrature node's weights should sum to 1.0";
    }
}

TEST_F(MatrixAssemblerBaseTest, test_neighbour_matrix_by_dof) {
    cnearest::MatrixAssemblerBase assembler(*assignment, *nearest_neighbours);
    auto v = assembler.nearest_neighbour_matrix_by_dof();
    ASSERT_EQ(v.rows(), 3 * 12);
    ASSERT_EQ(v.cols(), 3 * 5);
    for (int i = 0; i < 3 * 12; i++) {
        ASSERT_EQ(v.row(i).sum(), 1.0) << "Each quadrature node's weights should sum to 1.0";
    }
}

TEST_F(MatrixAssemblerBaseTest, test_assembly_disassembly_of_force_vector) {
    cnearest::MatrixAssemblerBase assembler(*assignment, *nearest_neighbours);

    cnearest::Vector v = cnearest::Vector::Random(15);

    auto decomposition = assembler.disassemble_vector_to_force_nodes(v);

    ASSERT_EQ(decomposition.size(), 5);
    for (auto& [node, index] : assignment->force_node_indicies()) {
        ASSERT_DOUBLE_EQ(decomposition.at(node).x(), v[3 * index + 0]);
        ASSERT_DOUBLE_EQ(decomposition.at(node).y(), v[3 * index + 1]);
        ASSERT_DOUBLE_EQ(decomposition.at(node).z(), v[3 * index + 2]);
    }

    auto recomposition = assembler.assemble_vector_from_nodal_values(decomposition);
    for (int i = 0; i < 15; i++) {
        ASSERT_EQ(recomposition(i), v(i));
    }
}

TEST_F(MatrixAssemblerBaseTest, test_assembly_disassembly_of_quadrature_vector) {
    cnearest::MatrixAssemblerBase assembler(*assignment, *nearest_neighbours);

    cnearest::Vector v = cnearest::Vector::Random(36);

    auto decomposition = assembler.disassemble_vector_to_quadrature_nodes(v);

    ASSERT_EQ(decomposition.size(), 12);
    for (auto& [node, index] : assignment->quadrature_node_indicies()) {
        ASSERT_DOUBLE_EQ(decomposition.at(node).x(), v[3 * index + 0]);
        ASSERT_DOUBLE_EQ(decomposition.at(node).y(), v[3 * index + 1]);
        ASSERT_DOUBLE_EQ(decomposition.at(node).z(), v[3 * index + 2]);
    }

    auto recomposition = assembler.assemble_vector_from_nodal_values(decomposition);
    for (int i = 0; i < 3 * 12; i++) {
        ASSERT_EQ(recomposition(i), v(i));
    }
}


TEST_F(MatrixAssemblerBaseTest, test_inverse_quadrature_node_sum) {
    cnearest::MatrixAssemblerBase assembler(*assignment, *nearest_neighbours);
    auto v = assembler.inverse_quadrature_node_sum_by_dof();
    ASSERT_EQ(v.size(), 3 * 5);
    for (auto& [node, index] : assignment->force_node_indicies()) {

        // elements 1 and 3 have 3 quadrature nodes, the others have 2 quadrature nodes
        double expected = (std::set({1, 3}).count(index) != 0) ? 1.0 / 3.0 : 1.0 / 2.0;

        EXPECT_DOUBLE_EQ(expected, v[3 * index + 0]);
        EXPECT_DOUBLE_EQ(expected, v[3 * index + 1]);
        EXPECT_DOUBLE_EQ(expected, v[3 * index + 2]);
    }
}