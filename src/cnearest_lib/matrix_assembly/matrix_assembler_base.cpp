#include "matrix_assembler_base.h"

namespace cnearest {
    MatrixAssemblerBase::MatrixAssemblerBase(
        const GlobalIndexAssignment& index_assignment,
                        const NearestNeighbourResults& neighbour_results): 
        _index_assignment(index_assignment), 
        _neighbour_results(neighbour_results) {

    }

    size_t MatrixAssemblerBase::number_of_force_nodes() const {
        return _index_assignment.force_node_indicies().size();
    }
}