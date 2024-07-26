#include "matrix_assembly/matrix_assembler_by_block_accumulation.h"
#include "nearest_neighbour/nearest_neighbour_detection.h"
#include "rigid_body.h"
#include "rigid_body_vector_operations.h"

namespace cnearest {

    /// <summary>
    /// Class to solve the resistance problem using the default tools and solvers
    /// The resistance problem is the case where the velocities are know and the forces must be calculated
    /// The only exposed option is the distributer to use in the case of ties
    /// All other options are encapsulated
    /// </summary>
    class ResistanceProblemSolver {
       public:
        /// <summary>
        /// define a resistance problem based on :
        /// a collection of rigid bodies,
        /// a regulasation epsilon,
        /// and an optional distributer to use in the case of ties
        /// </summary>
        ResistanceProblemSolver(const Collection<RigidBody>& bodies, double epsilon,
                              std::unique_ptr<iNearestNeighbourDistributer> distributer = nullptr);

        /// <summary>
        /// Given some 6dof speeds on the bodies, calculate the resulting 6dof forces
        /// </summary>
        std::map<const RigidBody*, SixDOFVector> solve_body_forces(
            const std::map<const RigidBody*, SixDOFVector>& speeds);

        
        /// <summary>
        /// Calculate the grand resistance matrix of one of the rigid bodies
        /// Works by calculating the force arising from the 6 loading directions, and using those as the columns of the matrix
        /// </summary>
        Matrix grand_resistance_matrix(const RigidBody&);

       private:
        const cnearest::Collection<RigidBody>& _bodies;
        GlobalIndexAssignment _index_assignment;
        NearestNeighbourResults _neighbour_results;
        MatrixAssemblerByBlockAccumulation<false> _assembler;
        Matrix _mean_stokeslet_matrix;
    };
}
