#pragma once
#include "definitions.h"
#include "matrix_assembler_base.h"

namespace cnearest {
    /// <summary>
    /// Assemble discretised resistance matrix by building a zero matrix then accumulating stokeslets at the correct index of the nearest neighbour
    /// </summary>
    class MatrixAssemblerByBlockAccumulation : MatrixAssemblerBase {
       public:
        MatrixAssemblerByBlockAccumulation(const GlobalIndexAssignment& index_assignment,
                                          const NearestNeighbourResults& neighbour_results, double epsilon);

        /// <summary>
        /// Generate the discretised resistance matrix by accumulating stokeslets (3M*3M)
        /// A
        /// Ref: j.jcp2017.12.008, eqn 26.
        /// This includes a 1/8pi multiplication
        /// </summary>
        Matrix generate_matrix() const;

       private:
        const double _epsilon;
    };
}