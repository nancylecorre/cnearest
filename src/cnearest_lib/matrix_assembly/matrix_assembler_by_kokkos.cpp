#include "matrix_assembler_by_kokkos.h"
#include "regularised_stokeslet.h"
#include<Kokkos_Core.hpp>

namespace cnearest {
    template <bool SortByDOF>
    MatrixAssemblerByKokkos<SortByDOF>::MatrixAssemblerByKokkos(
        const GlobalIndexAssignment& index_assignment, const NearestNeighbourResults& neighbour_results,
        double epsilon)
        : MatrixAssemblerBase<SortByDOF>(index_assignment, neighbour_results), _epsilon(epsilon) {}

    template <bool SortByDOF>
    Matrix MatrixAssemblerByKokkos<SortByDOF>::generate_summed_stokeslet_matrix() const {
        Matrix result = Matrix::Zero(3 * this->number_of_force_nodes(), 3 * this->number_of_force_nodes());

        Kokkos::initialize();


        int n = static_cast<int>(this->number_of_force_nodes());

        Kokkos::View<double* [3]> b("force positions", n);

        for (auto& [force_node, force_index] : this->_index_assignment.force_node_indicies()) {
            for (auto& [quadrature_node, _] : this->_index_assignment.quadrature_node_indicies()) {
                for (const NearestNeighbourResult::ForceNodeWithWeight& weight :
                     this->_neighbour_results.nearest_neighbour_mapping().at(quadrature_node)) {
                    int neighbour_index = this->_index_assignment.index(weight.force_node);
                    Matrix3 m = weight.weight * RegularisedStokeslet::stokeslet_tensor(
                                                    *force_node, *quadrature_node, _epsilon);
                    if (SortByDOF) {
                        result(force_index, neighbour_index) += m(0, 0);
                        result(force_index, neighbour_index + n) += m(0, 1);
                        result(force_index, neighbour_index + 2 * n) += m(0, 2);

                        result(force_index + n, neighbour_index) += m(1, 0);
                        result(force_index + n, neighbour_index + n) += m(1, 1);
                        result(force_index + n, neighbour_index + 2 * n) += m(1, 2);

                        result(force_index + 2 * n, neighbour_index) += m(2, 0);
                        result(force_index + 2 * n, neighbour_index + n) += m(2, 1);
                        result(force_index + 2 * n, neighbour_index + 2 * n) += m(2, 2);
                    } else {
                        result.block(3 * force_index, 3 * neighbour_index, 3, 3) += m;
                    }
                }
            }
        }
        return result / 8.0 / M_PI;
    }

    template <bool SortByDOF>
    Matrix MatrixAssemblerByKokkos<SortByDOF>::generate_mean_stokeslet_matrix() const {
        return generate_summed_stokeslet_matrix() * this->inverse_quadrature_node_sum_by_dof().asDiagonal();
    }

    // Explicit template instantiation
    template class MatrixAssemblerByKokkos<true>;
    template class MatrixAssemblerByKokkos<false>;
}
