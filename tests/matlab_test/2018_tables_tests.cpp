#include "discretisations/uniform_spherical_discretisation.h"
#include "global_index_assignment.h"
#include "gtest/gtest.h"
#include "linear_solve/linear_solver_lu_direct.h"
#include "matrix_assembly/matrix_assembler_by_block_accumulation.h"
#include "matrix_assembly/matrix_assembler_by_nn_multiplication.h"
#include "nearest_neighbour/nearest_neighbour_detection.h"
#include "rigid_body.h"
#include "rigid_body_vector_operations.h"

/// <summary>
/// This tests the results for a unit sphere against the tables 2a,2b, and 3 published in
/// A nearest-neighbour discretisation of the regularized stokeslet boundary integral equation
/// Smith 2018
/// http://dx.doi.org/10.1016/j.jcp.2017.12.008
/// </summary>

struct DataRow {
    int n;
    int q;
    double expected_force_error;
    double expected_moment_error;
    double expected_condition_number;
    double epsilon = 0.01;
};

class Paper2018TestTables : public ::testing::TestWithParam<DataRow> {};

TEST_P(Paper2018TestTables, TestForceMomentErrors) {
    auto& [n, q, expected_force_error, expected_moment_error, expected_condition_number,epsilon] = GetParam();
    // define sphere parameters
    double radius = 1.0;
    int force_resolution = n;
    int quadrature_resolution = q;

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
    cnearest::MatrixAssemblerByBlockAccumulation m(index_assignment, nearest_neighbour_results, epsilon);

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
    cnearest::Collection<const cnearest::Vector*> us = {&u_a, &u_b};
    auto f = cnearest::LinearSolverLUDirect().solve(mean_stokeslet_matrix, us);

    // decompose this to nodal forces
    auto nodal_forces_a = m.disassemble_vector_to_force_nodes(f[0]);
    auto nodal_forces_b = m.disassemble_vector_to_force_nodes(f[1]);

    // sum these to get the rigid body force
    auto summed_rigid_body_force = o.sum_nodal_forces(nodal_forces_a, nearest_neighbour_results);

    // compare with the predicted stokes force
    double force = summed_rigid_body_force.linear_component.x();
    double predicted_force = 6 * M_PI * radius * 1.0;

    double force_error = (force - predicted_force) / predicted_force;

    // sum these to get the rigid body moment
    summed_rigid_body_force = o.sum_nodal_forces(nodal_forces_b, nearest_neighbour_results);

    // compare with the predicted stokes force
    double moment = summed_rigid_body_force.angular_component.x();
    double predicted_moment = 8 * M_PI * radius * 1.0;

    double moment_error = (moment - predicted_moment) / predicted_moment;

    // accuracy of table 2
    ASSERT_NEAR(force_error, expected_force_error, 0.00005);
    ASSERT_NEAR(moment_error, expected_moment_error, 0.00005);
}

TEST_P(Paper2018TestTables, TestConditionNumbers) {
    auto& [n, q, expected_force_error, expected_moment_error, expected_condition_number, epsilon] =
        GetParam();

    //THESE ARE TOO SLOW FOR UNIT TESTS
    if (n > 12) return; 

    // define sphere parameters
    double radius = 1.0;
    int force_resolution = n;
    int quadrature_resolution = q;

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
    cnearest::MatrixAssemblerByBlockAccumulation m(index_assignment, nearest_neighbour_results, epsilon);

    cnearest::Matrix summed_stokeslet_matrix = m.generate_summed_stokeslet_matrix();

    Eigen::JacobiSVD<cnearest::Matrix> svd(summed_stokeslet_matrix);
    double condition_number =
        svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);


    // accuracy of table 3
    ASSERT_NEAR(condition_number, expected_condition_number, 0.0005);
}


INSTANTIATE_TEST_SUITE_P(
    Paper2018TestTables, Paper2018TestTables,
    ::testing::Values(DataRow({3, 12, 0.0147, 0.0300, 132.807}), DataRow({3, 24, 0.0052, 0.0086, 903.874}),
                      DataRow({3, 48, 0.0002, -0.0019, 415.837}), DataRow({3, 96, -0.0006, -0.0036, 292.569}),
                      DataRow({3, 192, -0.0012, -0.0047, 260.382}), DataRow({6, 12, 0.0166, 0.0378, 100.744}),
                      DataRow({6, 24, 0.0079, 0.0182, 242.112}), DataRow({6, 48, 0.0038, 0.0095, 904.001}),
                      DataRow({6, 96, 0.0022, 0.0063, 3344.192}), DataRow({6, 192, 0.0020, 0.0058, 1721.377}),
                      DataRow({12, 12, -0.1262, -0.2193, 8.129}), DataRow({12, 24, 0.0083, 0.0194, 218.433}),
                      DataRow({12, 48, 0.0043, 0.0109, 638.954}), DataRow({12, 96, 0.0027, 0.0078, 4823.278}),
                      DataRow({12, 192, 0.0025, 0.0074, 15637.232}), 
                      DataRow({24, 24, -0.0277, -0.0495, 38.309}), DataRow({24, 48, 0.0043, 0.0110, 617.308}),
                      DataRow({24, 96, 0.0028, 0.0080, 3696.167}),
                      DataRow({24, 192, 0.0025, 0.0075, 1870680.776}))

);
