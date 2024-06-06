#include "matrix_assembler_base.h"

namespace cnearest {

    template <bool SortByDOF>
    MatrixAssemblerBase<SortByDOF>::MatrixAssemblerBase<SortByDOF>(
        const GlobalIndexAssignment& index_assignment, const NearestNeighbourResults& neighbour_results)
        : _index_assignment(index_assignment), _neighbour_results(neighbour_results) {}

    template <bool SortByDOF>
    size_t MatrixAssemblerBase<SortByDOF>::number_of_force_nodes() const {
        return _index_assignment.force_node_indicies().size();
    }

    template <bool SortByDOF>
    size_t MatrixAssemblerBase<SortByDOF>::number_of_quadrature_nodes() const {
        return _index_assignment.quadrature_node_indicies().size();
    }

    template <bool SortByDOF>
    Vector MatrixAssemblerBase<SortByDOF>::concatenated_vector_of_force_positions() const {
        Vector result(3 * number_of_force_nodes());
        for (auto& [node, index] : _index_assignment.force_node_indicies()) {
            if (SortByDOF) {
                result(index) = node->position()(0);
                result(index + number_of_force_nodes()) = node->position()(1);
                result(index + 2 * number_of_force_nodes()) = node->position()(2);
            } else {
                result.segment(3 * index, 3) = node->position();
            }
        }
        return result;
    }

    template <bool SortByDOF>
    Vector MatrixAssemblerBase<SortByDOF>::concatenated_vector_of_quadrature_positions() const {
        Vector result(3 * number_of_quadrature_nodes());
        for (auto& [node, index] : _index_assignment.quadrature_node_indicies()) {
            if (SortByDOF) {
                result(index) = node->position()(0);
                result(index + number_of_quadrature_nodes()) = node->position()(1);
                result(index + 2 * number_of_quadrature_nodes()) = node->position()(2);
            } else {
                result.segment(3 * index, 3) = node->position();
            }
        }
        return result;
    }

    template <bool SortByDOF>
    Eigen::SparseMatrix<double> MatrixAssemblerBase<SortByDOF>::nearest_neighbour_matrix() const {
        Collection<Eigen::Triplet<double>> triplets;

        for (auto& [quadrature_node, row] : _index_assignment.quadrature_node_indicies()) {
            auto r = _neighbour_results.nearest_neighbour_mapping().at(quadrature_node);
            for (auto& weighting : r) {
                int col = _index_assignment.index(weighting.force_node);
                triplets.push_back({row, col, weighting.weight});
            }
        }
        Eigen::SparseMatrix<double> result(number_of_quadrature_nodes(), number_of_force_nodes());
        result.setFromTriplets(triplets.begin(), triplets.end());
        return result;
    }

    template <bool SortByDOF>
    Eigen::SparseMatrix<double> MatrixAssemblerBase<SortByDOF>::nearest_neighbour_matrix_by_dof() const {
        Collection<Eigen::Triplet<double>> triplets;

        for (auto& [quadrature_node, i] : _index_assignment.quadrature_node_indicies()) {
            int q_index = i;
            auto r = _neighbour_results.nearest_neighbour_mapping().at(quadrature_node);
            for (auto& weighting : r) {
                int f_index = _index_assignment.index(weighting.force_node);
                if (SortByDOF) {
                    triplets.push_back({q_index, f_index, weighting.weight});
                    triplets.push_back({q_index + (int)number_of_quadrature_nodes(),
                                        f_index + (int)number_of_force_nodes(), weighting.weight});
                    triplets.push_back({q_index + 2 * (int)number_of_quadrature_nodes(),
                                        f_index + 2 * (int)number_of_force_nodes(), weighting.weight});
                } else {
                    triplets.push_back({3 * q_index, 3 * f_index, weighting.weight});
                    triplets.push_back({3 * q_index+1, 3 * f_index+1, weighting.weight});
                    triplets.push_back({3 * q_index+2, 3 * f_index+2, weighting.weight});
                }
            }
        }
        Eigen::SparseMatrix<double> result(3 * number_of_quadrature_nodes(), 3 * number_of_force_nodes());
        result.setFromTriplets(triplets.begin(), triplets.end());
        return result;
    }

    template <bool SortByDOF>
    Vector MatrixAssemblerBase<SortByDOF>::assemble_vector_from_nodal_values(
        const std::map<const ForceNode*, Vector3>& nodal_values) const {
        Vector result = Vector::Zero(3 * number_of_force_nodes());
        for (auto& [node, segment] : nodal_values) {
            auto index = _index_assignment.index(node);
            if (SortByDOF) {
                result(index) = segment(0);
                result(index + number_of_force_nodes()) = segment(1);
                result(index + 2 * number_of_force_nodes()) = segment(2);
            } else {
                result.segment(3 * index, 3) = segment;
            }
        }
        return result;
    }

    template <bool SortByDOF>
    Vector MatrixAssemblerBase<SortByDOF>::assemble_vector_from_nodal_values(
        const std::map<const QuadratureNode*, Vector3>& nodal_values) const {
        Vector result = Vector::Zero(3 * number_of_quadrature_nodes());
        for (auto& [node, segment] : nodal_values) {
            auto index = _index_assignment.index(node);
            if (SortByDOF) {
                result(index) = segment(0);
                result(index + number_of_quadrature_nodes()) = segment(1);
                result(index + 2 * number_of_quadrature_nodes()) = segment(2);
            } else {
                result.segment(3 * index, 3) = segment;
            }
        }
        return result;
    }

    template <bool SortByDOF>
    std::map<const ForceNode*, Vector3> MatrixAssemblerBase<SortByDOF>::disassemble_vector_to_force_nodes(
        const Vector& vector) const {
        std::map<const ForceNode*, Vector3> result;
        for (auto& [node, index] : _index_assignment.force_node_indicies()) {
            if (SortByDOF) {
                result[node] = {vector(index), vector(index + number_of_force_nodes()),
                                vector(index + 2 * number_of_force_nodes())};
            } else {
                result[node] = vector.segment(3 * index, 3);
            }
        }
        return result;
    }

    template <bool SortByDOF>
    std::map<const QuadratureNode*, Vector3>
    MatrixAssemblerBase<SortByDOF>::disassemble_vector_to_quadrature_nodes(const Vector& vector) const {
        std::map<const QuadratureNode*, Vector3> result;
        for (auto& [node, index] : _index_assignment.quadrature_node_indicies()) {
            if (SortByDOF) {
                result[node] = {vector(index), vector(index + number_of_quadrature_nodes()),
                                vector(index + 2 * number_of_quadrature_nodes())};
            } else {
                result[node] = vector.segment(3 * index, 3);
            }
        }
        return result;
    }

    // Explicit template instantiation
    template class MatrixAssemblerBase<true>;
    template class MatrixAssemblerBase<false>;
}
