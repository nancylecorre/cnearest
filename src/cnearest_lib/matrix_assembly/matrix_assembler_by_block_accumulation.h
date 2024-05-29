#pragma once
#include "definitions.h"
#include "matrix_assembler_base.h"

namespace cnearest {
    /// <summary>
    /// Assemble global matrix by building a zero matrix then accumulating stokeslets at the correct index of the nearest neighbour
    /// </summary>
    class MatrixAssemblerByBlockAccumulation : MatrixAssemblerBase {
       public:
        MatrixAssemblerByBlockAccumulation(const GlobalIndexAssignment& index_assignment,
                                          const NearestNeighbourResults& neighbour_results, double epsilon);

        /// <summary>
        /// Generate the reduced matrix by accumulating stokeslets
        /// (3M*3M)
        /// This includes a 1/8pi multiplication
        /// </summary>
        Matrix generate_matrix() const;

       private:
        const double _epsilon;
    };
}