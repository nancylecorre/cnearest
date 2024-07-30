#include "resistance_problem_solver.h"

#include "linear_solve/linear_solver_lu_direct.h"

namespace cnearest {
    ResistanceProblemSolver::ResistanceProblemSolver(const Collection<cnearest::RigidBody>& bodies,
                                                 double epsilon,
                                                 std::unique_ptr<iNearestNeighbourDistributer> distributer)
        : _bodies(bodies),
          _index_assignment(_bodies),
          _neighbour_results(
              NearestNeighbourDetectionAlgorithm(std::move(distributer)).nearest_neighbours(_bodies)),
          _assembler(_index_assignment, _neighbour_results, epsilon),
          _mean_stokeslet_matrix(_assembler.generate_mean_stokeslet_matrix()) {}

    std::map<const RigidBody*, SixDOFVector> ResistanceProblemSolver::solve_body_forces(
        const std::map<const RigidBody*, SixDOFVector>& speeds) {
        std::map<const RigidBody*, SixDOFVector> result;

        std::map<const ForceNode*, Vector3> nodal_speeds;
        for (auto& [body, speed] : speeds) {
            cnearest::RigidBodyVectorOperations o(*body);

            auto s = o.determine_nodal_speeds(speed);
            nodal_speeds.insert(s.begin(), s.end());
        }

        auto u = _assembler.assemble_vector_from_nodal_values(nodal_speeds);

        auto f = LinearSolverLUDirect().solve(_mean_stokeslet_matrix, u);

        auto nodal_forces = _assembler.disassemble_vector_to_force_nodes(f);

        for (auto& body : _bodies) {
            cnearest::RigidBodyVectorOperations o(body);

            auto s = o.sum_nodal_forces(nodal_forces, _neighbour_results);
            result.insert({&body, s});
        }

        return result;
    }

    
    Matrix ResistanceProblemSolver::grand_resistance_matrix(const RigidBody& body) {
        cnearest::RigidBodyVectorOperations o(body);

        Vector speed_vectors[6];

        auto s = o.determine_nodal_speeds({{1, 0, 0}, {0, 0, 0}});
        speed_vectors[0] = _assembler.assemble_vector_from_nodal_values(s);

        s = o.determine_nodal_speeds({{0, 1, 0}, {0, 0, 0}});
        speed_vectors[1] = _assembler.assemble_vector_from_nodal_values(s);

        s = o.determine_nodal_speeds({{0, 0, 1}, {0, 0, 0}});
        speed_vectors[2] = _assembler.assemble_vector_from_nodal_values(s);

        s = o.determine_nodal_speeds({{0, 0, 0}, {1, 0, 0}});
        speed_vectors[3] = _assembler.assemble_vector_from_nodal_values(s);

        s = o.determine_nodal_speeds({{0, 0, 0}, {0, 1, 0}});
        speed_vectors[4] = _assembler.assemble_vector_from_nodal_values(s);

        s = o.determine_nodal_speeds({{0, 0, 0}, {0, 0, 1}});
        speed_vectors[5] = _assembler.assemble_vector_from_nodal_values(s);

        Collection<const Vector*> B = {&speed_vectors[0], &speed_vectors[1], &speed_vectors[2],
                                       &speed_vectors[3], &speed_vectors[4], &speed_vectors[5]};
        auto f = LinearSolverLUDirect().solve(_mean_stokeslet_matrix, B);

        Matrix result(6, 6);

        for (int i = 0; i < 6; i++) {
            auto nodal_forces = _assembler.disassemble_vector_to_force_nodes(f[i]);
            auto rigid_body_force = o.sum_nodal_forces(nodal_forces, _neighbour_results);
            result.row(i).segment<3>(0) = rigid_body_force.linear_component;
            result.row(i).segment<3>(3) = rigid_body_force.angular_component;
        }
        return result;

        }

}
