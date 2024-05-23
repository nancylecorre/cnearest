#include "matrix_assembler_base.h"

namespace cnearest {
    MatrixAssemblerBase::MatrixAssemblerBase(const GlobalIndexAssignment& index_assignment,
                                             const NearestNeighbourResults& neighbour_results)
        : _index_assignment(index_assignment), _neighbour_results(neighbour_results) {}

    size_t MatrixAssemblerBase::number_of_force_nodes() const {
        return _index_assignment.force_node_indicies().size();
    }

    size_t MatrixAssemblerBase::number_of_quadrature_nodes() const {
        return _index_assignment.quadrature_node_indicies().size();
    }

    Vector MatrixAssemblerBase::concatenated_vector_of_force_positions() const {
        Vector result(3 * number_of_force_nodes());
        for (auto& [node, index] : _index_assignment.force_node_indicies()) {
            result.segment(3 * index, 3) = node->position();
        }
        return result;
    }

    Vector MatrixAssemblerBase::concatenated_vector_of_quadrature_positions() const {
        Vector result(3 * number_of_quadrature_nodes());
        for (auto& [node, index] : _index_assignment.quadrature_node_indicies()) {
            result.segment(3 * index, 3) = node->position();
        }
        return result;
    }

    Eigen::SparseMatrix<double> MatrixAssemblerBase::nearest_neighbour_matrix() const {
        Collection<Eigen::Triplet<double>> triplets;

        for (auto& [quadrature_node, col] : _index_assignment.quadrature_node_indicies()) {
            auto r = _neighbour_results.nearest_neighbour_mapping().at(quadrature_node);
            for (auto& weighting : r) {
                int row = _index_assignment.index(weighting.force_node);
                triplets.push_back({row, col, weighting.weight});
            }
        }
        Eigen::SparseMatrix<double> result(number_of_force_nodes(), number_of_quadrature_nodes());
        result.setFromTriplets(triplets.begin(), triplets.end());
        return result;
    }

    Eigen::SparseMatrix<double> MatrixAssemblerBase::nearest_neighbour_matrix_by_dof() const {
        Collection<Eigen::Triplet<double>> triplets;

        for (auto& [quadrature_node, col] : _index_assignment.quadrature_node_indicies()) {
            auto r = _neighbour_results.nearest_neighbour_mapping().at(quadrature_node);
            for (auto& weighting : r) {
                int row = _index_assignment.index(weighting.force_node);
                triplets.push_back({3 * row + 0, 3 * col + 0, weighting.weight});
                triplets.push_back({3 * row + 1, 3 * col + 1, weighting.weight});
                triplets.push_back({3 * row + 2, 3 * col + 2, weighting.weight});
            }
        }
        Eigen::SparseMatrix<double> result(3 * number_of_force_nodes(), 3 * number_of_quadrature_nodes());
        result.setFromTriplets(triplets.begin(), triplets.end());
        return result;
    }

    Vector MatrixAssemblerBase::assemble_vector_from_nodal_values(
        const std::map<const ForceNode*, Vector3>& nodal_values) const {
        Vector result = Vector::Zero(3 * number_of_force_nodes());
        for (auto& [node, segment] : nodal_values) {
            auto index = _index_assignment.index(node);
            result.segment(3 * index, 3) = segment;
        }
        return result;
    }

    Vector MatrixAssemblerBase::assemble_vector_from_nodal_values(
        const std::map<const QuadratureNode*, Vector3>& nodal_values) const {
        Vector result = Vector::Zero(3 * number_of_quadrature_nodes());
        for (auto& [node, segment] : nodal_values) {
            auto index = _index_assignment.index(node);
            result.segment(3 * index, 3) = segment;
        }
        return result;
    }

    std::map<const ForceNode*, Vector3> MatrixAssemblerBase::disassemble_vector_to_force_nodes(
        const Vector& vector) const {
        std::map<const ForceNode*, Vector3> result;
        for (auto& [node, index] : _index_assignment.force_node_indicies()) {
            result[node] = vector.segment(3 * index, 3);
        }
        return result;
    }

    std::map<const QuadratureNode*, Vector3> MatrixAssemblerBase::disassemble_vector_to_quadrature_nodes(
        const Vector& vector) const {
        std::map<const QuadratureNode*, Vector3> result;
        for (auto& [node, index] : _index_assignment.quadrature_node_indicies()) {
            result[node] = vector.segment(3 * index, 3);
        }
        return result;
    }
}