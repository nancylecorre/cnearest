#include <optional>

#include "discretisations/straight_line_discretisation.h"
#include "global_index_assignment.h"
#include "gtest/gtest.h"
#include "matrix_assembly/matrix_assembler_by_nn_multiplication.h"
#include "matrix_assembly/matrix_assembler_by_block_accumulation.h"

class MatrixAssemblerTest : public ::testing::Test {
   protected:
    cnearest::Collection<cnearest::RigidBody> bodies;
    std::optional<cnearest::NearestNeighbourResults> nearest_neighbours;
    std::optional<cnearest::GlobalIndexAssignment> assignment;
    virtual void SetUp() {
        cnearest::StraightLineDiscretisation force_discretisation({0, 0, 1}, {0, 0, -1}, 3);
        cnearest::StraightLineDiscretisation quadrature_discretisation({0, 0, 1.05}, {0, 0, -1.05}, 10);

        bodies.push_back(cnearest::RigidBody(force_discretisation, quadrature_discretisation));

        assignment = cnearest::GlobalIndexAssignment(bodies);
        cnearest::NearestNeighbourDetectionAlgorithm algorithm;
        nearest_neighbours = algorithm.nearest_neighbours(bodies);
    };

    virtual void TearDown(){};
};

TEST_F(MatrixAssemblerTest, matrix_assembler_by_nn_multiplication) {
    cnearest::MatrixAssemblerByNNMultiplication assembler(*assignment, *nearest_neighbours, 1.0e-5);
    auto matrix = assembler.generate_summed_stokeslet_matrix();
    ASSERT_EQ(matrix.rows(), 3 * 5);
    ASSERT_EQ(matrix.cols(), 3 * 5);
    ASSERT_NEAR((matrix * 8 * M_PI)(0, 0), 27.09, 0.1);
}


TEST_F(MatrixAssemblerTest, matrix_assembler_by_block_accumulation) {
    cnearest::MatrixAssemblerByBlockAccumulation assembler(*assignment, *nearest_neighbours,
                                                          1.0e-5);
    auto matrix = assembler.generate_summed_stokeslet_matrix();
    ASSERT_EQ(matrix.rows(), 3 * 5);
    ASSERT_EQ(matrix.cols(), 3 * 5);
    ASSERT_NEAR((matrix * 8 * M_PI)(0, 0), 27.09, 0.1);
}

TEST_F(MatrixAssemblerTest, mean_matrix_assembler_by_nn_multiplication) {
    cnearest::MatrixAssemblerByNNMultiplication assembler(*assignment, *nearest_neighbours, 1.0e-5);
    auto matrix = assembler.generate_mean_stokeslet_matrix();
    ASSERT_EQ(matrix.rows(), 3 * 5);
    ASSERT_EQ(matrix.cols(), 3 * 5);
    ASSERT_NEAR((matrix * 8 * M_PI)(0, 0), 27.09 / 2.0, 0.1);
}

TEST_F(MatrixAssemblerTest, mean_matrix_assembler_by_block_accumulation) {
    cnearest::MatrixAssemblerByBlockAccumulation assembler(*assignment, *nearest_neighbours, 1.0e-5);
    auto matrix = assembler.generate_mean_stokeslet_matrix();
    ASSERT_EQ(matrix.rows(), 3 * 5);
    ASSERT_EQ(matrix.cols(), 3 * 5);
    ASSERT_NEAR((matrix * 8 * M_PI)(0, 0), 27.09 / 2.0, 0.1);
}

TEST_F(MatrixAssemblerTest, block_accumulation_equals_nn_multiplication) {

    cnearest::MatrixAssemblerByNNMultiplication nn_assembler(*assignment, *nearest_neighbours,
                                                          1.0e-5);
    auto nn_matrix = nn_assembler.generate_summed_stokeslet_matrix();
    cnearest::MatrixAssemblerByBlockAccumulation ba_assembler(*assignment, *nearest_neighbours,
                                                           1.0e-5);
    auto ba_matrix = ba_assembler.generate_summed_stokeslet_matrix();
    ASSERT_EQ(nn_matrix.norm(), ba_matrix.norm());
    ASSERT_EQ((nn_matrix - ba_matrix).norm(), 0.0);
}