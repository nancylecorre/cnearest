#include "matrix_assembler_by_kokkos_line.h"
#include "regularised_stokeslet.h"
#include<Kokkos_Core.hpp>

namespace cnearest {

    static void fill_stokeslet_results_for_node(const Kokkos::View<double* [3]>& f,
                                         size_t number_of_force_nodes, double _epsilon,
                                         double q_x, double q_y, double q_z,
                                         Kokkos::View<double* [9]>& r) {

        int n = static_cast<int>(number_of_force_nodes);

        // calculate the stokeslets

        Kokkos::parallel_for(
            "stokeslets",n, KOKKOS_LAMBDA(const int i) {
                const double offset_0 = f(i, 0) - q_x;
                const double offset_1 = f(i, 1) - q_y;
                const double offset_2 = f(i, 2) - q_z;
                const double r2 = offset_0 * offset_0 + offset_1 * offset_1 + offset_2 * offset_2;
                const double regularised_r2 = r2 + _epsilon * _epsilon;
                const double isotropic = (r2 + 2.0 * _epsilon * _epsilon) * std::pow(regularised_r2, -1.5);
                const double dyadic = std::pow(regularised_r2, -1.5);

                r(i, 0) = isotropic + dyadic * offset_0 * offset_0;
                r(i, 1) = dyadic * offset_1 * offset_0;
                r(i, 2) = dyadic * offset_2 * offset_0;

                r(i, 3) = dyadic * offset_0 * offset_1;
                r(i, 4) = isotropic + dyadic * offset_1 * offset_1;
                r(i, 5) = dyadic * offset_2 * offset_1;

                r(i, 6) = dyadic * offset_0 * offset_2;
                r(i, 7) = dyadic * offset_1 * offset_2;
                r(i, 8) = isotropic + dyadic * offset_2 * offset_2;
            });

        Kokkos::fence();
    }



    template <bool SortByDOF>
    MatrixAssemblerByKokkosLine<SortByDOF>::MatrixAssemblerByKokkosLine(
        const GlobalIndexAssignment& index_assignment, const NearestNeighbourResults& neighbour_results,
        double epsilon)
        : MatrixAssemblerBase<SortByDOF>(index_assignment, neighbour_results), _epsilon(epsilon) {}

    template <bool SortByDOF>
    Matrix MatrixAssemblerByKokkosLine<SortByDOF>::generate_summed_stokeslet_matrix() const {
        Matrix result = Matrix::Zero(3 * this->number_of_force_nodes(), 3 * this->number_of_force_nodes());

        int n = static_cast<int>(this->number_of_force_nodes());

        Kokkos::View<double* [3]> f("force positions", n);
        Kokkos::View<double* [9]> r("stokeslet_tensors", n);

        auto h_f = Kokkos::create_mirror_view(f);
        auto h_r = Kokkos::create_mirror_view(r);


        //populate the positions
        for (auto& [force_node, force_index] : this->_index_assignment.force_node_indicies()) {
            h_f(force_index, 0) = force_node->position()(0);
            h_f(force_index, 1) = force_node->position()(1);
            h_f(force_index, 2) = force_node->position()(2);
        }
        Kokkos::deep_copy(f, h_f);


        for (auto& [quadrature_node, _] : this->_index_assignment.quadrature_node_indicies()) {
            fill_stokeslet_results_for_node(f, n, _epsilon, quadrature_node->position()[0],
                                            quadrature_node->position()[1], quadrature_node->position()[2],
                                            r);

            Kokkos::deep_copy(h_r, r);

            // put these results into the matrix
            for (auto& [force_node, force_index] : this->_index_assignment.force_node_indicies()) {
                for (const NearestNeighbourResult::ForceNodeWithWeight& weight :
                     this->_neighbour_results.nearest_neighbour_mapping().at(quadrature_node)) {
                    int neighbour_index = this->_index_assignment.index(weight.force_node);
                    if (SortByDOF) {
                        result(force_index, neighbour_index) += weight.weight * h_r(force_index, 0);
                        result(force_index, neighbour_index + n) += weight.weight * h_r(force_index, 1);
                        result(force_index, neighbour_index + 2 * n) += weight.weight * h_r(force_index, 2);

                        result(force_index + n, neighbour_index) += weight.weight * h_r(force_index, 3);
                        result(force_index + n, neighbour_index + n) += weight.weight * h_r(force_index, 4);
                        result(force_index + n, neighbour_index + 2 * n) +=
                            weight.weight * h_r(force_index, 5);

                        result(force_index + 2 * n, neighbour_index) += weight.weight * h_r(force_index, 6);
                        result(force_index + 2 * n, neighbour_index + n) +=
                            weight.weight * h_r(force_index, 7);
                        result(force_index + 2 * n, neighbour_index + 2 * n) +=
                            weight.weight * h_r(force_index, 8);
                    } else {
                        result(3 * force_index, 3 * neighbour_index) += weight.weight * h_r(force_index, 0);
                        result(3 * force_index, 3 * neighbour_index + 1) +=
                            weight.weight * h_r(force_index, 1);
                        result(3 * force_index, 3 * neighbour_index + 2) +=
                            weight.weight * h_r(force_index, 2);

                        result(3 * force_index + 1, 3 * neighbour_index) +=
                            weight.weight * h_r(force_index, 3);
                        result(3 * force_index + 1, 3 * neighbour_index + 1) +=
                            weight.weight * h_r(force_index, 4);
                        result(3 * force_index + 1, 3 * neighbour_index + 2) +=
                            weight.weight * h_r(force_index, 5);

                        result(3 * force_index + 2, 3 * neighbour_index) +=
                            weight.weight * h_r(force_index, 6);
                        result(3 * force_index + 2, 3 * neighbour_index + 1) +=
                            weight.weight * h_r(force_index, 7);
                        result(3 * force_index + 2, 3 * neighbour_index + 2) +=
                            weight.weight * h_r(force_index, 8);
                    }
                }
            }
        }
        return result / 8.0 / M_PI;
    }

    template <bool SortByDOF>
    Matrix MatrixAssemblerByKokkosLine<SortByDOF>::generate_mean_stokeslet_matrix() const {
        return generate_summed_stokeslet_matrix() * this->inverse_quadrature_node_sum_by_dof().asDiagonal();
    }

    // Explicit template instantiation
    template class MatrixAssemblerByKokkosLine<true>;
    template class MatrixAssemblerByKokkosLine<false>;
    }
