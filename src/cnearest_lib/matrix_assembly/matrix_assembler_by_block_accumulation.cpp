#include "matrix_assembler_by_block_accumulation.h"

namespace cnearest {
    MatrixAssemblerByBlockAccumulation::MatrixAssemblerByBlockAccumulation(
        const GlobalIndexAssignment& index_assignment, const NearestNeighbourResults& neighbour_results,
        double epsilon)
        : MatrixAssemblerBase(index_assignment, neighbour_results), _epsilon(epsilon) {}

    Matrix MatrixAssemblerByBlockAccumulation::generate_matrix() const {
        Matrix result = Matrix::Zero(3 * number_of_force_nodes(), 3 * number_of_force_nodes());

        for (auto& [force_node, force_index] : _index_assignment.force_node_indicies()) {
            for (auto& [quadrature_node, _] : _index_assignment.quadrature_node_indicies()) {

                for (const NearestNeighbourResult::ForceNodeWithWeight& weight :
                     _neighbour_results.nearest_neighbour_mapping().at(quadrature_node)) {
                    int neighbour_index = _index_assignment.index(weight.force_node);
                    result.block(3 * force_index, 3 * neighbour_index, 3, 3) +=
                        weight.weight * Matrix3::Identity();
                }
            }
        }
        return result / 8.0 / M_PI;
    }
}
