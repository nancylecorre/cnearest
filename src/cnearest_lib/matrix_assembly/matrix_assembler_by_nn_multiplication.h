#pragma once
#include "definitions.h"
#include "matrix_assembler_base.h"

namespace cnearest {
    /// <summary>
    /// Assemble discretised resistance by building a block matrix of stokeslets, then reducing it with the
    /// nearest neighbour matrix
    /// </summary>
    /// <typeparam name="SortByDOF"> The sorting off the indicies in the global matrix. 
    ///  If false then all the dofs of a single node will be adjacent (partitioning the matrix into 3*3 blocks)
    ///  If true then all dofs in the x/y/z direction will be adjacent (partitioning the matrix into M*M blocks)</typeparam>
    template <bool SortByDOF = false>
    class MatrixAssemblerByNNMultiplication : MatrixAssemblerBase<SortByDOF> {
       public:
        MatrixAssemblerByNNMultiplication(const GlobalIndexAssignment& index_assignment,
                                          const NearestNeighbourResults& neighbour_results, double epsilon);

        /// <summary>
        /// Generate the discretised resistance matrix by multiplying the stokeslets by the nearest neighbour
        /// matrix (3M*3M)
        /// A
        /// Ref: j.jcp2017.12.008, eqn 26.
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
