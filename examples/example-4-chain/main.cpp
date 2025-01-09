#include "discretisations/uniform_spherical_discretisation.h"
#include "rigid_body.h"
#include "nearest_neighbour/nearest_neighbour_detection.h"
#include "nearest_neighbour_distributers.h"
#include "global_index_assignment.h"
#include "matrix_assembly/matrix_assembler_by_nn_multiplication.h"
#include "matrix_assembly/matrix_assembler_by_block_accumulation.h"
#include "matrix_assembler_by_kokkos.h"
#include "rigid_body_vector_operations.h"
#include "linear_solve/linear_solver_lu_direct.h"
#include "table_printing.h"
#include <chrono>
#include<Kokkos_Core.hpp>

int main(int argc, char *argv[]) {
    Kokkos::initialize();

    print_table_header({{"N", 8}, {"Q", 8}, {"Force Error", 15}, {"Moment Error", 15}, {"Time (s)", 15}});

    auto start = std::chrono::high_resolution_clock::now();
    // define sphere parameters
    double radius = 1.0;
    int force_resolution = 12;
    int quadrature_resolution = 192;

    // Define regularisation
    double epsilon = 0.01;

    // Create discretisation
    cnearest::UniformSphericalDiscretisation force_discretisation(radius, force_resolution);
    cnearest::UniformSphericalDiscretisation quadrature_discretisation(radius, quadrature_resolution);

    // Create a collection with one rigid body
    cnearest::Collection<cnearest::RigidBody> bodies{
        cnearest::RigidBody(force_discretisation, quadrature_discretisation),
        cnearest::RigidBody(force_discretisation, quadrature_discretisation, {2.1,0.1,0}),
        cnearest::RigidBody(force_discretisation, quadrature_discretisation, {4.2, 0.2, 0}),
        cnearest::RigidBody(force_discretisation, quadrature_discretisation, {6.3, 0.3, 0}),
        cnearest::RigidBody(force_discretisation, quadrature_discretisation, {8.4, 0.4, 0})};

    // Detect the nearest neighbours
    cnearest::NearestNeighbourDetectionAlgorithm algorithm(std::make_unique<cnearest::NearestNeighbourDistributerFirst>());
    auto nearest_neighbour_results = algorithm.nearest_neighbours(bodies);

    // Assign global ids to the nodes (trivial with one rigid body)
    cnearest::GlobalIndexAssignment index_assignment(bodies);

    // Construct the assembled matrix
    cnearest::MatrixAssemblerByKokkos<false> m(index_assignment, nearest_neighbour_results, epsilon);

    cnearest::Matrix mean_stokeslet_matrix = m.generate_mean_stokeslet_matrix();

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
    cnearest::Collection<const cnearest::Vector *> us({&u_a, &u_b});
    auto f = cnearest::LinearSolverLUDirect().solve(mean_stokeslet_matrix, us);

    // decompose this to nodal forces
    auto nodal_forces_a = m.disassemble_vector_to_force_nodes(f[0]);
    auto nodal_forces_b = m.disassemble_vector_to_force_nodes(f[1]);

    // sum these to get the rigid body force
    auto summed_rigid_body_force = o.sum_nodal_forces(nodal_forces_a, nearest_neighbour_results);

    // compare with the predicted stokes force
    double force = summed_rigid_body_force.linear_component.x();
    double predicted_force = 6 * M_PI * radius * 1.0;

    double force_error = (force - predicted_force) / ((predicted_force));

    // sum these to get the rigid body moment
    summed_rigid_body_force = o.sum_nodal_forces(nodal_forces_b, nearest_neighbour_results);

    // compare with the predicted stokes force
    double moment = summed_rigid_body_force.angular_component.x();
    double predicted_moment = 8 * M_PI * radius * 1.0;

    double moment_error = (moment - predicted_moment) / ((predicted_moment));

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;

    print_table_row({{(double)index_assignment.force_node_indicies().size(), 0, 8},
                     {(double)index_assignment.quadrature_node_indicies().size(), 0, 8},
                     {force_error, 8, 15},
                     {moment_error, 8, 15},
                     {duration, 8, 15}});
}
    