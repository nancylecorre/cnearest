#include "matrix_assembler_by_nn_multiplication.h"


namespace cnearest {
    MatrixAssemblerByNNMultiplication::MatrixAssemblerByNNMultiplication(
        const GlobalIndexAssignment& index_assignment, const NearestNeighbourResults& neighbour_results, double epsilon)
        : MatrixAssemblerBase(index_assignment, neighbour_results), _epsilon(epsilon) {}


    Matrix MatrixAssemblerByNNMultiplication::generate_matrix() const {
        return generate_stokeslet_matrix() * nearest_neighbour_matrix_by_dof()/8.0/M_PI;
    }
    Matrix MatrixAssemblerByNNMultiplication::generate_stokeslet_matrix() const { 
        
        Matrix result(3 * number_of_force_nodes(), 3 * number_of_quadrature_nodes());

        for (auto& [force_node, force_index] : _index_assignment.force_node_indicies()) {
            for (auto& [quadrature_node, quadrature_index] : _index_assignment.quadrature_node_indicies()) {
                result.block(3 * force_index, 3 * quadrature_index, 3, 3) = Matrix3::Identity();
            }
        }

        return result;
    }
}
