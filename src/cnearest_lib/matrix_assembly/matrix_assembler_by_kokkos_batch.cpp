#include "matrix_assembler_by_kokkos_batch.h"
#include "regularised_stokeslet.h"
#include<Kokkos_Core.hpp>

namespace cnearest {

    typedef Kokkos::MDRangePolicy<Kokkos::Rank<2>> mdrange_policy;
    typedef Kokkos::MDRangePolicy<Kokkos::DefaultHostExecutionSpace, Kokkos::Rank<2>> mdrange_policy_host;

    static void fill_stokeslet_results_for_batch(const Kokkos::View<double* [3]>& f,
                                         size_t number_of_force_nodes, double _epsilon,
                                         const std::vector<const QuadratureNode*> quadrature_nodes,
                                         Kokkos::View<double** [9]>& r) {

        int n = static_cast<int>(number_of_force_nodes);
        int m = static_cast<int>(quadrature_nodes.size());

        Kokkos::View<double* [3]> q("quadrature positions", m);
        auto h_q = Kokkos::create_mirror_view(q);

        
        Kokkos::parallel_for(
            "populate the quadrature positions", mdrange_policy_host({0, 0}, {m, 3}),
            KOKKOS_LAMBDA(const int i, const int j) { h_q(i, j) = quadrature_nodes[i]->position()(j); });
        Kokkos::deep_copy(q, h_q);

        // calculate the stokeslets

        Kokkos::parallel_for(
            "stokeslets", mdrange_policy({0, 0}, {n, m}), KOKKOS_LAMBDA(const int i, const int j) {
                const double offset_0 = f(i, 0) - q(j, 0);
                const double offset_1 = f(i, 1) - q(j, 1);
                const double offset_2 = f(i, 2) - q(j, 2);
                const double r2 = offset_0 * offset_0 + offset_1 * offset_1 + offset_2 * offset_2;
                const double regularised_r2 = r2 + _epsilon * _epsilon;
                const double isotropic = (r2 + 2.0 * _epsilon * _epsilon) * std::pow(regularised_r2, -1.5);
                const double dyadic = std::pow(regularised_r2, -1.5);

                r(i, j, 0) = isotropic + dyadic * offset_0 * offset_0;
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
    }



    template <bool SortByDOF>
    MatrixAssemblerByKokkosBatch<SortByDOF>::MatrixAssemblerByKokkosBatch(
        const GlobalIndexAssignment& index_assignment, const NearestNeighbourResults& neighbour_results,
        double epsilon, int batch_size)
        : MatrixAssemblerBase<SortByDOF>(index_assignment, neighbour_results), _epsilon(epsilon), _batch_size(batch_size) {}

    template <bool SortByDOF>
    Matrix MatrixAssemblerByKokkosBatch<SortByDOF>::generate_summed_stokeslet_matrix() const {
        Matrix result = Matrix::Zero(3 * this->number_of_force_nodes(), 3 * this->number_of_force_nodes());

        int n = static_cast<int>(this->number_of_force_nodes());

        Kokkos::View<double* [3]> f("force positions", n);
        Kokkos::View<double** [9]> r("stokeslet_tensors", n,_batch_size);

        auto h_f = Kokkos::create_mirror_view(f);
        auto h_r = Kokkos::create_mirror_view(r);


        //populate the positions
        for (auto& [force_node, force_index] : this->_index_assignment.force_node_indicies()) {
            h_f(force_index, 0) = force_node->position()(0);
            h_f(force_index, 1) = force_node->position()(1);
            h_f(force_index, 2) = force_node->position()(2);
        }
        Kokkos::deep_copy(f, h_f);

        //batch the quadrature nodes
        std::vector<const QuadratureNode*> nodes;
        for (auto const& node : this->_index_assignment.quadrature_node_indicies())
            nodes.push_back(node.first);
        std::vector<std::vector<const QuadratureNode*>> batches; 
        for (size_t i = 0; i < nodes.size(); i += _batch_size) {
            auto last = std::min(nodes.size(), i + _batch_size);
            batches.emplace_back(nodes.begin() + i, nodes.begin() + last);
        }

        for (auto& batch : batches) {
            fill_stokeslet_results_for_batch(f, n, _epsilon, batch, r);

            Kokkos::deep_copy(h_r, r);

            // put these results into the matrix
            for (auto& [force_node, force_index] : this->_index_assignment.force_node_indicies()) {
                for (int quadrature_index = 0; quadrature_index < batch.size(); quadrature_index++) {
                    for (const NearestNeighbourResult::ForceNodeWithWeight& weight :
                         this->_neighbour_results.nearest_neighbour_mapping().at(batch[quadrature_index])) {
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
        }
        return result / 8.0 / M_PI;
    }

    template <bool SortByDOF>
    Matrix MatrixAssemblerByKokkosBatch<SortByDOF>::generate_mean_stokeslet_matrix() const {
        return generate_summed_stokeslet_matrix() * this->inverse_quadrature_node_sum_by_dof().asDiagonal();
    }

    // Explicit template instantiation
    template class MatrixAssemblerByKokkosBatch<true>;
    template class MatrixAssemblerByKokkosBatch<false>;
}
