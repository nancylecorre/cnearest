#include <Kokkos_Core.hpp>

#include "matrix_assembler_by_kokkos.h"
#include "regularised_stokeslet.h"

namespace cnearest {

struct MatrixContribution {
        const QuadratureNode* quadrature_node;
        const double weight;
        const int neighbour_index;
    };

    typedef Kokkos::MDRangePolicy<Kokkos::Rank<2>> mdrange_policy;
    typedef Kokkos::MDRangePolicy<Kokkos::DefaultHostExecutionSpace, Kokkos::Rank<2>> mdrange_policy_host;
    typedef Kokkos::RangePolicy<Kokkos::DefaultHostExecutionSpace> range_policy_host;

    static void fill_stokeslet_matrix_for_batch(const Kokkos::View<double* [3]>& f,
                                                 size_t number_of_force_nodes, double _epsilon,
                                                 const std::vector<MatrixContribution>& batch,
                                                 Kokkos::View<double**, Kokkos::MemoryTraits<Kokkos::Atomic>>& r) {
        int n = static_cast<int>(number_of_force_nodes);
        int m = static_cast<int>(batch.size());

        Kokkos::View<double* [3]> q("quadrature positions", m);
        auto h_q = Kokkos::create_mirror_view(q);

        Kokkos::View<double*> w("weights", m);
        auto h_w = Kokkos::create_mirror_view(w);

        Kokkos::View<int*> neigh("nearest indicies", m);
        auto h_n = Kokkos::create_mirror_view(neigh);

        Kokkos::parallel_for(
            "populate the quadrature positions", range_policy_host(0, m),
            KOKKOS_LAMBDA(const int i) { h_w(i) = batch[i].weight; });
        Kokkos::deep_copy(w, h_w);

        
        Kokkos::parallel_for(
            "populate the weights", range_policy_host(0, m),
            KOKKOS_LAMBDA(const int i) { h_n(i) = batch[i].neighbour_index; });
        Kokkos::deep_copy(neigh, h_n);

        
        Kokkos::parallel_for(
            "populate the quadrature positions", mdrange_policy_host({0, 0}, {m, 3}),
            KOKKOS_LAMBDA(const int i, const int j) { h_q(i, j) = batch[i].quadrature_node->position()(j); });
        Kokkos::deep_copy(q, h_q);

        // calculate the stokeslets

        Kokkos::parallel_for(
            "stokeslets", mdrange_policy({0, 0}, {n, m}), KOKKOS_LAMBDA(const int i, const int j) {
                const double offset_0 = f(i, 0) - q(j, 0);
                const double offset_1 = f(i, 1) - q(j, 1);
                const double offset_2 = f(i, 2) - q(j, 2);
                const double weight = w(j);
                const int neighbour_index = neigh(j);
                const double r2 = offset_0 * offset_0 + offset_1 * offset_1 + offset_2 * offset_2;
                const double regularised_r2 = r2 + _epsilon * _epsilon;
                const double isotropic = (r2 + 2.0 * _epsilon * _epsilon) * std::pow(regularised_r2, -1.5);
                const double dyadic = std::pow(regularised_r2, -1.5);

                r(3 * i, 3 * neighbour_index) += weight * isotropic + weight * dyadic * offset_0 * offset_0;
                r(3 * i, 3 * neighbour_index+1) += weight * dyadic * offset_1 * offset_0;
                r(3 * i, 3 * neighbour_index+2) += weight * dyadic * offset_2 * offset_0;

                r(3 * i+1, 3 * neighbour_index) += weight * dyadic * offset_0 * offset_1;
                r(3 * i+1, 3 * neighbour_index+1) += weight * isotropic + weight * dyadic * offset_1 * offset_1;
                r(3 * i+1, 3 * neighbour_index+2) += weight * dyadic * offset_2 * offset_1;

                r(3 * i+2, 3 * neighbour_index) += weight * dyadic * offset_0 * offset_2;
                r(3 * i+2, 3 * neighbour_index+1) += weight * dyadic * offset_1 * offset_2;
                r(3 * i+2, 3 * neighbour_index+2) += weight * isotropic + weight * dyadic * offset_2 * offset_2;
            });

        Kokkos::fence();
    }

    template <bool SortByDOF>
    MatrixAssemblerByKokkos2<SortByDOF>::MatrixAssemblerByKokkos2(
        const GlobalIndexAssignment& index_assignment, const NearestNeighbourResults& neighbour_results,
        double epsilon, int batch_size)
        : MatrixAssemblerBase<SortByDOF>(index_assignment, neighbour_results),
          _epsilon(epsilon),
          _batch_size(batch_size) {}

    template <bool SortByDOF>
    Matrix MatrixAssemblerByKokkos2<SortByDOF>::generate_summed_stokeslet_matrix() const {
        Matrix result = Matrix::Zero(3 * this->number_of_force_nodes(), 3 * this->number_of_force_nodes());

        int n = static_cast<int>(this->number_of_force_nodes());

        Kokkos::View<double* [3]> f("force positions", n);
        Kokkos::View<double**, Kokkos::MemoryTraits<Kokkos::Atomic>> r(
            "stokeslet_matrix", 3 * this->number_of_force_nodes(), 3 * this->number_of_force_nodes());

        auto h_f = Kokkos::create_mirror_view(f);
        auto h_r = Kokkos::create_mirror_view(r);

        // populate the positions
        for (auto& [force_node, force_index] : this->_index_assignment.force_node_indicies()) {
            h_f(force_index, 0) = force_node->position()(0);
            h_f(force_index, 1) = force_node->position()(1);
            h_f(force_index, 2) = force_node->position()(2);
        }
        Kokkos::deep_copy(f, h_f);

        //populate the matrix contributions
        std::vector<MatrixContribution> contributions;
        for (auto const& [quad_node, _] : this->_index_assignment.quadrature_node_indicies()) {
            for (const NearestNeighbourResult::ForceNodeWithWeight& weight :
                 this->_neighbour_results.nearest_neighbour_mapping().at(quad_node)) {
                int neighbour_index = this->_index_assignment.index(weight.force_node);
                contributions.push_back({quad_node, weight.weight, neighbour_index});
            }
        }
        // batch the matrix_contributions
        std::vector<std::vector<MatrixContribution>> batches;
        for (size_t i = 0; i < contributions.size(); i += _batch_size) {
            auto last = std::min(contributions.size(), i + _batch_size);
            batches.emplace_back(contributions.begin() + i, contributions.begin() + last);
        }

        for (auto& batch : batches) {
            fill_stokeslet_matrix_for_batch(f, n, _epsilon, batch, r);
        }
        Kokkos::deep_copy(h_r, r);

        for (int i = 0; i < 3 * this->number_of_force_nodes(); i++) {
            for (int j = 0; j < 3 * this->number_of_force_nodes(); j++) {
                result(i, j) = h_r(i, j);
            }

        }
        return result / 8.0 / M_PI;
    }

    template <bool SortByDOF>
    Matrix MatrixAssemblerByKokkos2<SortByDOF>::generate_mean_stokeslet_matrix() const {
        return generate_summed_stokeslet_matrix() * this->inverse_quadrature_node_sum_by_dof().asDiagonal();
    }

    // Explicit template instantiation
    template class MatrixAssemblerByKokkos2<true>;
    template class MatrixAssemblerByKokkos2<false>;
}
