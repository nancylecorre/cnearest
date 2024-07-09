#include "discretisations/uniform_spherical_discretisation.h"
#include "rigid_body.h"
#include "nearest_neighbour/nearest_neighbour_detection.h"
#include "global_index_assignment.h"
#include "matrix_assembly/matrix_assembler_by_nn_multiplication.h"
#include "matrix_assembly/matrix_assembler_by_block_accumulation.h"
#include "rigid_body_vector_operations.h"
#include "linear_solve/linear_solver_lu_direct.h"
#include "table_printing.h"
#include <chrono>

int main(int argc, char *argv[]) {

    //Print a table header
    std::cout << centre("N", 8) << " | " << centre("Q", 8) << " | " << centre("Force Error", 15) << " | "
              << centre("Moment Error", 15) << " | " << centre("CN summed", 15) << " | "
              << centre("CN mean", 15) << " | " << centre("Time (s)", 15)
              
               << "\n";
    std::cout << std::string(16+15 * 5 + 2 * 9, '-') << "\n";


    for (int n : {3, 6, 12 , 24}) {
        for (int q : {12, 24,48, 96, 192 }) {
            if (q < n) continue;

            auto start = std::chrono::high_resolution_clock::now();
            // define sphere parameters
            double radius = 1.0;
            int force_resolution = n;        
            int quadrature_resolution = q;  

            // Define regularisation
            double epsilon = 0.01;


            // Create discretisation
            cnearest::UniformSphericalDiscretisation force_discretisation(radius, force_resolution);
            cnearest::UniformSphericalDiscretisation quadrature_discretisation(radius, quadrature_resolution);

            // Create a collection with one rigid body
            cnearest::Collection<cnearest::RigidBody> bodies{
                cnearest::RigidBody(force_discretisation, quadrature_discretisation)};

            // Detect the nearest neighbours
            cnearest::NearestNeighbourDetectionAlgorithm algorithm;
            auto nearest_neighbour_results = algorithm.nearest_neighbours(bodies);

            // Assign global ids to the nodes (trivial with one rigid body)
            cnearest::GlobalIndexAssignment index_assignment(bodies);

            // Construct the assembled matrix
            cnearest::MatrixAssemblerByBlockAccumulation m(index_assignment, nearest_neighbour_results,
                                                          epsilon);

            cnearest::Matrix stokeslet_matrix = m.generate_summed_stokeslet_matrix();

            Eigen::JacobiSVD<cnearest::Matrix> svd(stokeslet_matrix);
            double condition_number_summed = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

            cnearest::Matrix mean_stokeslet_matrix = m.generate_mean_stokeslet_matrix();

            svd = Eigen::JacobiSVD<cnearest::Matrix>(mean_stokeslet_matrix);
            double condition_number_mean =
                svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

            assert(!mean_stokeslet_matrix.hasNaN());

            // give each node the same speed
            cnearest::RigidBodyVectorOperations o(bodies[0]);

            // case a motion in linear direction
            cnearest::Vector3 linear_speed = {1, 0, 0};
            cnearest::Vector3 angular_speed = {0, 0, 0};
            auto nodal_speeds_a = o.determine_nodal_speeds({linear_speed, angular_speed});

            // case b motion in angular direction
            linear_speed = {0, 0, 0};
            angular_speed = {1, 0, 0};
            auto nodal_speeds_b = o.determine_nodal_speeds({linear_speed, angular_speed});

            // build a unified vector of these speeds
            auto u_a = m.assemble_vector_from_nodal_values(nodal_speeds_a);
            auto u_b = m.assemble_vector_from_nodal_values(nodal_speeds_b);

            // perform an lu decomposition to facilitate the solve
            // solve to get a unified vector of all the forces
            cnearest::Collection<const cnearest::Vector *> us= {&u_a, &u_b};
            auto f = cnearest::LinearSolverLUDirect().solve(mean_stokeslet_matrix, us);

            // decompose this to nodal forces
            auto nodal_forces_a = m.disassemble_vector_to_force_nodes(f[0]);
            auto nodal_forces_b = m.disassemble_vector_to_force_nodes(f[1]);

            // sum these to get the rigid body force
            auto summed_rigid_body_force = o.sum_nodal_forces(nodal_forces_a);

            // compare with the predicted stokes force
            double force = summed_rigid_body_force.linear_component.x();
            double predicted_force = 6 * M_PI * radius * 1.0;

            double force_error = std::abs(force - predicted_force) / ((force + predicted_force) / 2.0);

            
            // sum these to get the rigid body moment
            summed_rigid_body_force = o.sum_nodal_forces(nodal_forces_b);

            // compare with the predicted stokes force
            double moment = summed_rigid_body_force.angular_component.x();
            double predicted_moment = 8 * M_PI * radius * 1.0;

            double moment_error = std::abs(moment - predicted_moment) / ((moment + predicted_moment) / 2.0);

            auto stop = std::chrono::high_resolution_clock::now();    
            auto duration = duration_cast<std::chrono::milliseconds>(stop - start).count()/1000.0;
            
            //Print a table row
            std::cout << prd(bodies[0].force_nodes().size(), 0, 8) << " | "
                      << prd(bodies[0].quadrature_nodes().size(), 0, 8) << " | " << prd(force_error, 8, 15)
                      << " | " << prd(moment_error, 8, 15) << " | " << prd(condition_number_summed, 8, 15) << " | " << prd(condition_number_mean, 8, 15) 
                      << " | " << prd(duration, 8, 15)
                      << "\n";
        }
    }

}
