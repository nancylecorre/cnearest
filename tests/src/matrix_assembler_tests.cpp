#include <optional>

#include "discretisations/line_discretisation.h"
#include "global_index_assignment.h"
#include "gtest/gtest.h"
#include "matrix_assembly/matrix_assembler_by_nn_multiplication.h"
#include "matrix_assembly/matrix_assembler_by_block_accumulation.h"

class MatrixAssemblerBaseTest : public ::testing::Test {
   protected:
    std::optional<cnearest::NearestNeighbourResults> nearest_neighbours;
    std::optional<cnearest::GlobalIndexAssignment> assignment;
    virtual void SetUp() {
        cnearest::LineDiscretisation force_discretisation({0, 0, 1}, {0, 0, -1}, 3);
        cnearest::LineDiscretisation quadrature_discretisation({0, 0, 1.05}, {0, 0, -1.05}, 10);

        cnearest::Collection<cnearest::RigidBody> bodies;
        bodies.push_back(cnearest::RigidBody(force_discretisation, quadrature_discretisation));

        assignment = cnearest::GlobalIndexAssignment(bodies);
        cnearest::NearestNeighbourDetectionAlgorithm algorithm;
        nearest_neighbours = algorithm.nearest_neighbours(bodies);
    };

    virtual void TearDown(){};
};

TEST_F(MatrixAssemblerBaseTest, matrix_assembler_by_nn_multiplication) {
    cnearest::MatrixAssemblerByNNMultiplication assembler(assignment.value(), nearest_neighbours.value(), 1.0e-5);
    auto matrix = assembler.generate_matrix();
    ASSERT_EQ(matrix.rows(), 3 * 5);
    ASSERT_EQ(matrix.cols(), 3 * 5);
}


TEST_F(MatrixAssemblerBaseTest, matrix_assembler_by_block_accumulation) {
    cnearest::MatrixAssemblerByBlockAccumulation assembler(assignment.value(), nearest_neighbours.value(),
                                                          1.0e-5);
    auto matrix = assembler.generate_matrix();
    ASSERT_EQ(matrix.rows(), 3 * 5);
    ASSERT_EQ(matrix.cols(), 3 * 5);
}