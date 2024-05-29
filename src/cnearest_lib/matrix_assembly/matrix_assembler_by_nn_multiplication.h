#pragma once
#include "definitions.h"
#include "matrix_assembler_base.h"


namespace cnearest {
    /// <summary>
    /// Assemble global matrix by building a block matrix of stokeslets, then reducing it with the nearest neighbour matrix
    /// </summary>
    class MatrixAssemblerByNNMultiplication : MatrixAssemblerBase {
       public:
        MatrixAssemblerByNNMultiplication(const GlobalIndexAssignment& index_assignment,
                                          const NearestNeighbourResults& neighbour_results,
                                          double epsilon);

        /// <summary>
        /// Generate the reduced matrix by multiplying the stokeslets by the nearest neighbour matrix
        /// (3M*3M)
        /// This includes a 1/8pi multiplication
        /// </summary>
        Matrix generate_matrix() const;

        /// <summary>
        /// Builds a large dense matrix of all the stokeslets (3M*3Q), this will later get reduced with the
        /// nearest neighbours.
        /// S
        /// </summary>
        Matrix generate_stokeslet_matrix() const;

       private:
        const double _epsilon;
    };
}