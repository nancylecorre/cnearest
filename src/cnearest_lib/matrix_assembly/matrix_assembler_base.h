#pragma once
#include "nearest_neighbour/nearest_neighbour_detection.h"
#include "global_index_assignment.h"
#include <Eigen/Sparse>

namespace cnearest {
    /// <summary>
    /// Tools to help assemble global matrix based on the indicies and neighbour findings.
    /// No responsibility for storage of inputs.
    /// </summary>
    class MatrixAssemblerBase {
       public:
        MatrixAssemblerBase(const GlobalIndexAssignment& index_assignment,
                            const NearestNeighbourResults& neighbour_results);

        /// <summary>
        /// Q
        /// Ref: j.jcp2017.12.008, Chapter 4.
        /// </summary>
        size_t number_of_force_nodes() const;

        /// <summary>
        /// M
        /// Ref: j.jcp2017.12.008, Chapter 4.
        /// </summary>
        size_t number_of_quadrature_nodes() const;

        /// <summary>
        /// Vector of force points (length 3Q)
        /// X
        /// Ref: j.jcp2017.12.008, eqn 20.
        /// </summary>
        Vector concatenated_vector_of_force_positions() const;

        /// <summary>
        /// Vector of quadrature points (length 3M)
        /// x
        /// Ref: j.jcp2017.12.008, eqn 20.
        /// </summary>
        Vector concatenated_vector_of_quadrature_positions() const;

        /// <summary>
        /// Nearest neighbour operator matrix by node (size QxN)
        /// V[q,n]
        /// Ref: j.jcp2017.12.008, eqn 21.
        /// Note: this may include non-unity weighted entries, not present in the
        /// original paper
        /// </summary>
        Eigen::SparseMatrix<double> nearest_neighbour_matrix() const;

        /// <summary>
        /// Nearest neighbour operator matrix by dof (size 3Qx3N)
        /// V[q,n]
        /// Ref: j.jcp2017.12.008, eqn 21.
        /// Note: this may include non-unity weighted entries, not present in the
        /// original paper
        /// </summary>
        Eigen::SparseMatrix<double> nearest_neighbour_matrix_by_dof() const;

        /// <summary>
        /// converting from multiple 3d vectors on nodes, to a global vector (length 3Q)
        /// </summary>
        Vector assemble_vector_from_nodal_values(
            const std::map<const ForceNode*, Vector3>& nodal_values) const;

        /// <summary>
        /// converting from multiple 3d vectors on nodes, to a global vector (length 3M)
        /// </summary>
        Vector assemble_vector_from_nodal_values(
            const std::map<const QuadratureNode*, Vector3>& nodal_values) const;

        /// <summary>
        /// interpreting a global vector (length 3Q) as multiple 3d vectors on force nodes
        /// </summary>
        std::map<const ForceNode*, Vector3> disassemble_vector_to_force_nodes(
            const Vector& vector) const;

        /// <summary>
        /// interpreting a global vector (length 3M) as multiple 3d vectors on quadrature
        /// nodes
        /// </summary>
        std::map<const QuadratureNode*, Vector3> disassemble_vector_to_quadrature_nodes(
            const Vector& vector) const;

       private:
        const GlobalIndexAssignment& _index_assignment;
        const NearestNeighbourResults& _neighbour_results;
    };
}
