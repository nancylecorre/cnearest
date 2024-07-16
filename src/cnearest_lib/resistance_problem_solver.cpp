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

}
