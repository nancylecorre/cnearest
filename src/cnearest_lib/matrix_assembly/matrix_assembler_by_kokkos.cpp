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

        int n = static_cast<int>(this->number_of_force_nodes());
        int m = static_cast<int>(this->number_of_quadrature_nodes());

        Kokkos::View<double* [3]> f("force positions", n);
        Kokkos::View<double* [3]> q("quadrature positions", m);
        Kokkos::View<double** [9]> r("stokeslet_tensors", n,m);

        auto h_f = Kokkos::create_mirror_view(f);
        auto h_q = Kokkos::create_mirror_view(q);
        auto h_r = Kokkos::create_mirror_view(r);

        //populate the positions
        for (auto& [force_node, force_index] : this->_index_assignment.force_node_indicies()) {
            h_f(force_index, 0) = force_node->position()(0);
            h_f(force_index, 1) = force_node->position()(1);
            h_f(force_index, 2) = force_node->position()(2);
        }
        for (auto& [quadrature_node, quadrature_index] : this->_index_assignment.quadrature_node_indicies()) {
            h_q(quadrature_index, 0) = quadrature_node->position()(0);
            h_q(quadrature_index, 1) = quadrature_node->position()(1);
            h_q(quadrature_index, 2) = quadrature_node->position()(2);
        }
        Kokkos::deep_copy(f, h_f);
        Kokkos::deep_copy(q, h_q);

        //calculate the stokeslets

        typedef Kokkos::MDRangePolicy<Kokkos::Rank<2> > mdrange_policy;
        Kokkos::parallel_for(
            "stokeslets", mdrange_policy({0, 0}, {n, m}), KOKKOS_LAMBDA(const int i, const int j) {
                const double offset_0 = f(i, 0) - q(j, 0);
                const double offset_1 = f(i, 1) - q(j, 1);
                const double offset_2 = f(i, 2) - q(j, 2);
                const double r2 = offset_0 * offset_0 + offset_1 * offset_1 + offset_2 * offset_2;
                const double regularised_r2 = r2 + _epsilon * _epsilon;
                const double isotropic = (r2 + 2.0 * _epsilon * _epsilon) * std::pow(regularised_r2, -1.5);
                const double dyadic = std::pow(regularised_r2, -1.5);


                r(i, j, 0) = isotropic + dyadic*offset_0*offset_0;
                r(i, j, 1) = dyadic * offset_1 * offset_0;
                r(i, j, 2) = dyadic * offset_2 * offset_0;

                r(i, j, 3) = dyadic * offset_0 * offset_1;
                r(i, j, 4) = isotropic + dyadic * offset_1 * offset_1;
                r(i, j, 5) = dyadic * offset_2 * offset_1;

                r(i, j, 6) = dyadic * offset_0 * offset_2;
                r(i, j, 7) = dyadic * offset_1 * offset_2;
                r(i, j, 8) = isotropic + dyadic * offset_2 * offset_2;
            });

        Kokkos::fence();
        Kokkos::deep_copy(h_r, r);

        //put these results into the matrix
        for (auto& [force_node, force_index] : this->_index_assignment.force_node_indicies()) {
            for (auto& [quadrature_node, quadrature_index] :
                 this->_index_assignment.quadrature_node_indicies()) {
                for (const NearestNeighbourResult::ForceNodeWithWeight& weight :
                     this->_neighbour_results.nearest_neighbour_mapping().at(quadrature_node)) {
                    int neighbour_index = this->_index_assignment.index(weight.force_node);
                    if (SortByDOF) {
                        result(force_index, neighbour_index) +=
                            weight.weight * h_r(force_index, quadrature_index, 0);
                        result(force_index, neighbour_index + n) +=
                            weight.weight * h_r(force_index, quadrature_index, 1);
                        result(force_index, neighbour_index + 2 * n) +=
                            weight.weight * h_r(force_index, quadrature_index, 2);

                        result(force_index + n, neighbour_index) +=
                            weight.weight * h_r(force_index, quadrature_index, 3);
                        result(force_index + n, neighbour_index + n) +=
                            weight.weight * h_r(force_index, quadrature_index, 4);
                        result(force_index + n, neighbour_index + 2 * n) +=
                            weight.weight * h_r(force_index, quadrature_index, 5);

                        result(force_index + 2 * n, neighbour_index) +=
                            weight.weight * h_r(force_index, quadrature_index, 6);
                        result(force_index + 2 * n, neighbour_index + n) +=
                            weight.weight * h_r(force_index, quadrature_index, 7);
                        result(force_index + 2 * n, neighbour_index + 2 * n) +=
                            weight.weight * h_r(force_index, quadrature_index, 8);
                    } else {
                        result(3 * force_index, 3 * neighbour_index) +=
                            weight.weight * h_r(force_index, quadrature_index, 0);
                        result(3 * force_index, 3 * neighbour_index + 1) +=
                            weight.weight * h_r(force_index, quadrature_index, 1);
                        result(3 * force_index, 3 * neighbour_index + 2) +=
                            weight.weight * h_r(force_index, quadrature_index, 2);

                        result(3 * force_index + 1, 3 * neighbour_index) +=
                            weight.weight * r(force_index, quadrature_index, 3);
                        result(3 * force_index + 1, 3 * neighbour_index + 1) +=
                            weight.weight * r(force_index, quadrature_index, 4);
                        result(3 * force_index + 1, 3 * neighbour_index + 2) +=
                            weight.weight * r(force_index, quadrature_index, 5);

                        result(3 * force_index + 2, 3 * neighbour_index) +=
                            weight.weight * r(force_index, quadrature_index, 6);
                        result(3 * force_index + 2, 3 * neighbour_index + 1) +=
                            weight.weight * r(force_index, quadrature_index, 7);
                        result(3 * force_index + 2, 3 * neighbour_index + 2) +=
                            weight.weight * r(force_index, quadrature_index, 8);
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
