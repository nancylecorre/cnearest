#pragma once
#include "definitions.h"
#include "matrix_assembler_base.h"

namespace cnearest {
    /// <summary>
    /// </summary>
    /// <typeparam name="SortByDOF"> The sorting off the indicies in the global matrix.
    ///  If false then all the dofs of a single node will be adjacent (partitioning the matrix into 3*3
    ///  blocks)
    /// If true then all dofs in the x/y/z direction will be adjacent (partitioning the matrix into
    ///  M*M blocks)</typeparam>
    template <bool SortByDOF = false>
    class MatrixAssemblerByKokkos : public MatrixAssemblerBase<SortByDOF> {
       public:
        MatrixAssemblerByKokkos(const GlobalIndexAssignment& index_assignment,
                                           const NearestNeighbourResults& neighbour_results, double epsilon);

        /// <summary>
        /// Generate the discretised resistance matrix by accumulating stokeslets (3M*3M)
        /// A
        /// Ref: j.jcp2017.12.008, eqn 26.
        /// This includes a 1/8pi multiplication
        /// </summary>
        Matrix generate_summed_stokeslet_matrix() const;

        /// <summary>
        /// Generate the mean stokeslet matrix
        /// Divide the summed stokeslet matrix by the number of stokeslets assigned to the force node
        /// </summary>
        Matrix generate_mean_stokeslet_matrix() const;

       private:
        const double _epsilon;
    };
}
