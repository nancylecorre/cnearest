#include "discretisations/line_discretisation.h"
#include "global_index_assignment.h"
#include "matrix_assembly/matrix_assembler_base.h"
#include <optional>
#include "gtest/gtest.h"

class MatrixAssemblerBaseTest : public ::testing::Test {
   protected:
    std::optional<cnearest::NearestNeighbourResults> nearest_neighbours;
    std::optional<cnearest::GlobalIndexAssignment> assignment;
    virtual void SetUp(){
        cnearest::LineDiscretisation force_discretisation({0, 0, 1}, {0, 0, -1}, 3);
        cnearest::LineDiscretisation quadrature_discretisation({0, 0, 1.05},
                                                               {0, 0, -1.05}, 10);

        cnearest::Collection<cnearest::RigidBody> bodies;
        bodies.push_back(
            cnearest::RigidBody(force_discretisation, quadrature_discretisation));

        assignment = cnearest::GlobalIndexAssignment(bodies);
        cnearest::NearestNeighbourDetectionAlgorithm algorithm;
        nearest_neighbours = algorithm.nearest_neighbours(bodies);
    
    };

    virtual void TearDown(){};
};

TEST_F(MatrixAssemblerBaseTest, test_q) {
    cnearest::MatrixAssemblerBase assembler(assignment.value(), nearest_neighbours.value());
    ASSERT_EQ(assembler.number_of_force_nodes(), 5);
}
