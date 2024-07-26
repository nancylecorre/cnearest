#include "discretisations/uniform_spherical_discretisation.h"
#include "gtest/gtest.h"
#include "resistance_problem_solver.h"
#include "rigid_body.h"

struct ResistanceProblemDataRow {
    int n;
    int q;
    double expected_force_error;
    double expected_moment_error;
    double epsilon = 0.01;
};

class ResistanceSolverTest : public ::testing::TestWithParam<ResistanceProblemDataRow> {};

TEST_P(ResistanceSolverTest, TestSolvingResistanceProblemForOneSphere) {
    auto& [n, q, expected_force_error, expected_moment_error, epsilon] =
        GetParam();
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

    cnearest::ResistanceProblemSolver solver(bodies, epsilon);

    // motion in linear direction
    cnearest::SixDOFVector linear_speed = {{1, 0, 0}, {0, 0, 0}};
    std::map<const cnearest::RigidBody*, cnearest::SixDOFVector> speeds = {{&bodies[0], linear_speed}};
    auto forces = solver.solve_body_forces(speeds);

    double force = forces[&bodies[0]].linear_component.x();
    double predicted_force = 6 * M_PI * radius * 1.0;
    double force_error = (force - predicted_force) / predicted_force;

    // motion in angular direction
    cnearest::SixDOFVector angular_speed = {{0, 0, 0}, {1, 0, 0}};
    speeds = {{&bodies[0], angular_speed}};
    forces = solver.solve_body_forces(speeds);

    double moment = forces[&bodies[0]].angular_component.x();
    double predicted_moment = 8 * M_PI * radius * 1.0;
    double moment_error = (moment - predicted_moment) / predicted_moment;

    ASSERT_NEAR(force_error, expected_force_error, 0.00005);
    ASSERT_NEAR(moment_error, expected_moment_error, 0.00005);
}

TEST_P(ResistanceSolverTest, TestSolvingResistanceProblemForTwoSpheres) {
    auto& [n, q, expected_force_error, expected_moment_error, epsilon] = GetParam();
    // define sphere parameters
    double radius = 1.0;
    int force_resolution = n;
    int quadrature_resolution = q;

    // Create discretisation
    cnearest::UniformSphericalDiscretisation force_discretisation(radius, force_resolution);
    cnearest::UniformSphericalDiscretisation quadrature_discretisation(radius, quadrature_resolution);

    // Create a collection with one rigid body
    cnearest::Collection<cnearest::RigidBody> bodies{
        cnearest::RigidBody(force_discretisation, quadrature_discretisation),
        cnearest::RigidBody(force_discretisation, quadrature_discretisation, {10,0,0})};

    cnearest::ResistanceProblemSolver solver(bodies, epsilon);

    // motion in linear direction
    cnearest::SixDOFVector linear_speed = {{1, 0, 0}, {0, 0, 0}};
    std::map<const cnearest::RigidBody*, cnearest::SixDOFVector> speeds = {{&bodies[0], linear_speed}};
    auto forces = solver.solve_body_forces(speeds);

    double force = forces[&bodies[0]].linear_component.x();
    double predicted_force = 6 * M_PI * radius * 1.0;

    // motion in angular direction
    cnearest::SixDOFVector angular_speed = {{0, 0, 0}, {1, 0, 0}};
    speeds = {{&bodies[0], angular_speed}};
    forces = solver.solve_body_forces(speeds);

    double moment = forces[&bodies[0]].angular_component.x();
    double predicted_moment = 8 * M_PI * radius * 1.0;

    ASSERT_NEAR(force, predicted_force, 2.5);
    ASSERT_NEAR(moment, predicted_moment, 2.5);
}


TEST_P(ResistanceSolverTest, TestSolvingResistanceMatrixForOneSphere) {
    auto& [n, q, expected_force_error, expected_moment_error, epsilon] = GetParam();
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

    cnearest::ResistanceProblemSolver solver(bodies, epsilon);

    auto matrix = solver.grand_resistance_matrix(bodies[0]);

}

TEST_P(ResistanceSolverTest, TestSolvingResistanceMatrixForTwoSpheres) {
    auto& [n, q, expected_force_error, expected_moment_error, epsilon] = GetParam();
    // define sphere parameters
    double radius = 1.0;
    int force_resolution = n;
    int quadrature_resolution = q;

    // Create discretisation
    cnearest::UniformSphericalDiscretisation force_discretisation(radius, force_resolution);
    cnearest::UniformSphericalDiscretisation quadrature_discretisation(radius, quadrature_resolution);

    // Create a collection with one rigid body
    cnearest::Collection<cnearest::RigidBody> bodies{
        cnearest::RigidBody(force_discretisation, quadrature_discretisation),
        cnearest::RigidBody(force_discretisation, quadrature_discretisation, {10, 0, 0})};

    cnearest::ResistanceProblemSolver solver(bodies, epsilon);

    auto matrix0 = solver.grand_resistance_matrix(bodies[0]);
    auto matrix1 = solver.grand_resistance_matrix(bodies[1]);
}


// these values come from the table in the 2018 paper
INSTANTIATE_TEST_SUITE_P(ResistanceSolverTest, ResistanceSolverTest,
                         ::testing::Values(ResistanceProblemDataRow({3, 12, 0.0147, 0.0300}),
                                           ResistanceProblemDataRow({3, 24, 0.0052, 0.0086}),
                                           ResistanceProblemDataRow({3, 48, 0.0002, -0.0019}),
                                           ResistanceProblemDataRow({3, 96, -0.0006, -0.0036}),
                                           ResistanceProblemDataRow({6, 12, 0.0166, 0.0378}))

);
