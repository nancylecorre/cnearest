#include "matrix_assembler_by_nn_multiplication.h"
#include "regularised_stokeslet.h"


namespace cnearest {
    template <bool SortByDOF>
    MatrixAssemblerByNNMultiplication<SortByDOF>::MatrixAssemblerByNNMultiplication(
        const GlobalIndexAssignment& index_assignment, const NearestNeighbourResults& neighbour_results, double epsilon)
        : MatrixAssemblerBase<SortByDOF>(index_assignment, neighbour_results), _epsilon(epsilon) {}

    
    template <bool SortByDOF>
    Matrix MatrixAssemblerByNNMultiplication<SortByDOF>::generate_matrix() const {
        return generate_stokeslet_matrix() * this->nearest_neighbour_matrix_by_dof() / 8.0 / M_PI;
    }
    template <bool SortByDOF>
    Matrix MatrixAssemblerByNNMultiplication<SortByDOF>::generate_stokeslet_matrix() const { 
        
        int n = this->number_of_force_nodes();
        int q = this->number_of_quadrature_nodes();
        Matrix result(3 * this->number_of_force_nodes(), 3 * this->number_of_quadrature_nodes());

        for (auto& [force_node, force_index] : this->_index_assignment.force_node_indicies()) {
            for (auto& [quadrature_node, quadrature_index] :
                 this->_index_assignment.quadrature_node_indicies()) {

                Matrix3 m = RegularisedStokeslet::stokeslet_tensor(*force_node, *quadrature_node, _epsilon);
                if (SortByDOF) {
                    result(force_index, quadrature_index) = m(0, 0);
                    result(force_index, quadrature_index + q) = m(0, 1);
                    result(force_index, quadrature_index + 2 * q) = m(0, 2);

                    result(force_index + n, quadrature_index) = m(1, 0);
                    result(force_index + n, quadrature_index + q) = m(1, 1);
                    result(force_index + n, quadrature_index + 2 * q) = m(1, 2);

                    result(force_index + 2 * n, quadrature_index) = m(2, 0);
                    result(force_index + 2 * n, quadrature_index + q) = m(2, 1);
                    result(force_index + 2 * n, quadrature_index + 2 * q) = m(2, 2);
                } else {
                    result.block(3 * force_index, 3 * quadrature_index, 3, 3) = m;
                }

                
            }
        }

        return result;
    }

    
    // Explicit template instantiation
    template class MatrixAssemblerByNNMultiplication<true>;
    template class MatrixAssemblerByNNMultiplication<false>;
}
