#include "gtest/gtest.h"
#include "read_matrix.h"
#include "custom_distrbuter.h"
#include "discretisations/uniform_spherical_discretisation.h"
#include "discretisations/empirical_discretisation.h"
#include "matrix_assembly/matrix_assembler_by_nn_multiplication.h"
#include "matrix_assembly/matrix_assembler_by_block_accumulation.h"
#include "nearest_neighbour/nearest_neighbour_distributers.h"
#include "regularised_stokeslet.h"

const std::string data_folder = "../../../../../tests/matlab_test/data/";

class MatlabTest : public ::testing::Test {
   protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(MatlabTest, test_loading_data) {
    cnearest::Matrix A3 = read_csv(data_folder + "A3.dat", 162, 162);
    ASSERT_EQ(A3.rows(), 162);
    ASSERT_EQ(A3.cols(), 162);
    ASSERT_FALSE(A3.hasNaN());

    cnearest::Matrix r1 = read_csv(data_folder + "r1.dat", 54, 54);
    ASSERT_EQ(r1.rows(), 54);
    ASSERT_EQ(r1.cols(), 54);
    ASSERT_FALSE(r1.hasNaN());

    cnearest::Matrix r2 = read_csv(data_folder + "r2.dat", 54, 54);
    ASSERT_EQ(r2.rows(), 54);
    ASSERT_EQ(r2.cols(), 54);
    ASSERT_FALSE(r2.hasNaN());

    cnearest::Matrix r3 = read_csv(data_folder + "r3.dat", 54, 54);
    ASSERT_EQ(r3.rows(), 54);
    ASSERT_EQ(r3.cols(), 54);
    ASSERT_FALSE(r3.hasNaN());

    
    cnearest::Vector xQuad = read_csv(data_folder + "xQuad.dat", 2592);
    ASSERT_EQ(xQuad.size(), 2592);
    ASSERT_FALSE(xQuad.hasNaN());

    
    cnearest::Vector xTrac = read_csv(data_folder + "xTrac.dat", 162);
    ASSERT_EQ(xTrac.size(), 162);
    ASSERT_FALSE(xTrac.hasNaN());

    
    cnearest::Matrix SxX = read_csv(data_folder + "_S_xX.dat", 162, 2592);
    ASSERT_EQ(SxX.rows(), 162);
    ASSERT_EQ(SxX.cols(), 2592);
    ASSERT_FALSE(SxX.hasNaN());
}

TEST_F(MatlabTest, DISABLED_test_trac_positions) {

    double radius = 5.0;

    cnearest::Vector xTrac = read_csv(data_folder + "xTrac.dat", 162);

    cnearest::UniformSphericalDiscretisation d(radius, 3);
    auto positions = d.positions();

    
    for (int i = 0; i < positions.size(); i++) {
        EXPECT_NEAR(
            pow(xTrac(i), 2) + pow(xTrac(54+i), 2) + pow(xTrac(108+i), 2),
            radius * radius, 1.0e-9) << i;
    }
    
    ASSERT_EQ(xTrac.size(), positions.size()*3);

    for (int i = 0; i < positions.size(); i++) {
        EXPECT_DOUBLE_EQ(xTrac(i), positions[i].x());
    }



}


TEST_F(MatlabTest, test_single_stokeslet_maths) {
    double epsilon = 0.02;

    cnearest::Vector xTrac = read_csv(data_folder + "xTrac.dat", 162);
    cnearest::Vector3 first_force_position = {xTrac(0), xTrac(54), xTrac(108)};
    cnearest::ForceNode first_force_node(first_force_position);

    cnearest::Vector xQuad = read_csv(data_folder + "xQuad.dat", 2592);
    cnearest::Vector3 first_quad_position = {xQuad(0), xQuad(864), xQuad(864 + 864)};
    cnearest::QuadratureNode first_quad_node(first_quad_position);

    cnearest::Matrix SxX = read_csv(data_folder + "_S_xX.dat", 162, 2592);

    auto stokeslet = cnearest::RegularisedStokeslet::stokeslet_tensor(first_force_node, first_quad_node, epsilon)/8/M_PI;


    EXPECT_NEAR(SxX(0, 0), stokeslet(0, 0), 1.0e-4);
    EXPECT_NEAR(SxX(0, 864), stokeslet(0, 1), 1.0e-4);
    EXPECT_NEAR(SxX(0, 1728), stokeslet(0, 2), 1.0e-4);
    EXPECT_NEAR(SxX(54, 0), stokeslet(1, 0), 1.0e-4);
    EXPECT_NEAR(SxX(54, 864), stokeslet(1, 1), 1.0e-4);
    EXPECT_NEAR(SxX(54, 1728), stokeslet(1, 2), 1.0e-4);
    EXPECT_NEAR(SxX(108, 0), stokeslet(2, 0), 1.0e-4);
    EXPECT_NEAR(SxX(108, 864), stokeslet(2, 1), 1.0e-4);
    EXPECT_NEAR(SxX(108, 1728), stokeslet(2, 2), 1.0e-4);
}


TEST_F(MatlabTest, test_single_stokeslet_maths_2) {
    double epsilon = 0.02;

    cnearest::Vector xTrac = read_csv(data_folder + "xTrac.dat", 162);
    cnearest::Vector3 first_force_position = {xTrac(0), xTrac(54), xTrac(108)};
    cnearest::ForceNode first_force_node(first_force_position);

    cnearest::Vector xQuad = read_csv(data_folder + "xQuad.dat", 2592);
    cnearest::Vector3 first_quad_position = {xQuad(1812-864-864), xQuad(1812 - 864), xQuad(1812)};
    cnearest::QuadratureNode first_quad_node(first_quad_position);

    cnearest::Matrix SxX = read_csv(data_folder + "_S_xX.dat", 162, 2592);

    auto stokeslet =
        cnearest::RegularisedStokeslet::stokeslet_tensor(first_force_node, first_quad_node, epsilon) / 8 /
        M_PI;

    EXPECT_NEAR(SxX(0, 1812 - 864 - 864), stokeslet(0, 0), 1.0e-4);
    EXPECT_NEAR(SxX(0, 1812 - 864), stokeslet(0, 1), 1.0e-4);
    EXPECT_NEAR(SxX(0, 1812), stokeslet(0, 2), 1.0e-4);
    EXPECT_NEAR(SxX(54, 1812 - 864 - 864), stokeslet(1, 0), 1.0e-4);
    EXPECT_NEAR(SxX(54, 1812 - 864), stokeslet(1, 1), 1.0e-4);
    EXPECT_NEAR(SxX(54, 1812), stokeslet(1, 2), 1.0e-4);
    EXPECT_NEAR(SxX(108, 1812 - 864 - 864), stokeslet(2, 0), 1.0e-4);
    EXPECT_NEAR(SxX(108, 1812 - 864), stokeslet(2, 1), 1.0e-4);
    EXPECT_NEAR(SxX(108, 1812), stokeslet(2, 2), 1.0e-4);
}

TEST_F(MatlabTest, test_stokeslet_matrix_assembly) {

    double epsilon = 0.02;

    cnearest::Vector xTrac = read_csv(data_folder + "xTrac.dat", 162);
    cnearest::Collection<cnearest::Vector3> force_positions;
    for (int i = 0; i < 54; i++) {
        force_positions.push_back({xTrac(i), xTrac(i + 54), xTrac(i + 108)});
    }
    cnearest::EmpiricalDiscretisation force_discretisation(force_positions);

    
    cnearest::Vector xQuad = read_csv(data_folder + "xQuad.dat", 2592);
    cnearest::Collection<cnearest::Vector3> quad_positions;
    for (int i = 0; i < 864; i++) {
        quad_positions.push_back({xQuad(i), xQuad(i + 864), xQuad(i + 864 + 864)});
    }
    cnearest::EmpiricalDiscretisation quad_discretisation(quad_positions);

    
    cnearest::Matrix SxX = read_csv(data_folder + "_S_xX.dat", 162, 2592);


    // Create a collection with one rigid body
    cnearest::Collection<cnearest::RigidBody> bodies{
        cnearest::RigidBody(force_discretisation, quad_discretisation)};

    // Assign global ids to the nodes (trivial with one rigid body)
    cnearest::GlobalIndexAssignment index_assignment(bodies);

    // Detect the nearest neighbours
    cnearest::NearestNeighbourDetectionAlgorithm algorithm = matlab_matching_detection(index_assignment);
    auto nearest_neighbour_results = algorithm.nearest_neighbours(bodies);


    // Construct the assembled matrix
    cnearest::MatrixAssemblerByNNMultiplication<true> m(index_assignment, nearest_neighbour_results, epsilon);

    cnearest::Matrix stokeslet_matrix = m.generate_stokeslet_matrix()/8/M_PI;
    
    for (int i = 0; i < SxX.rows(); i++) {
        for (int j = 0; j < SxX.cols(); j++) {
            EXPECT_NEAR(SxX(i, j), stokeslet_matrix(i, j), 1.0e-4) << "Difference at:" << i << "," << j;
        }
    }

}


TEST_F(MatlabTest, test_nearest_neighbour_matrix) {
    double epsilon = 0.02;

    cnearest::Vector xTrac = read_csv(data_folder + "xTrac.dat", 162);
    cnearest::Collection<cnearest::Vector3> force_positions;
    for (int i = 0; i < 54; i++) {
        force_positions.push_back({xTrac(i), xTrac(i + 54), xTrac(i + 108)});
    }
    cnearest::EmpiricalDiscretisation force_discretisation(force_positions);

    cnearest::Vector xQuad = read_csv(data_folder + "xQuad.dat", 2592);
    cnearest::Collection<cnearest::Vector3> quad_positions;
    for (int i = 0; i < 864; i++) {
        quad_positions.push_back({xQuad(i), xQuad(i + 864), xQuad(i + 864 + 864)});
    }
    cnearest::EmpiricalDiscretisation quad_discretisation(quad_positions);

    
    cnearest::Matrix NClosest1 = read_csv(data_folder + "NClosest1.dat", 864, 54);
    cnearest::Matrix nu = read_csv(data_folder + "nu.dat", 2592, 162);

    // Create a collection with one rigid body
    cnearest::Collection<cnearest::RigidBody> bodies{
        cnearest::RigidBody(force_discretisation, quad_discretisation)};

    // Assign global ids to the nodes (trivial with one rigid body)
    cnearest::GlobalIndexAssignment index_assignment(bodies);


    // Detect the nearest neighbours
    cnearest::NearestNeighbourDetectionAlgorithm algorithm = matlab_matching_detection(index_assignment);
    auto nearest_neighbour_results = algorithm.nearest_neighbours(bodies);


    // Construct the nearest neighbour matrix
    cnearest::MatrixAssemblerBase<true> m(index_assignment, nearest_neighbour_results);

    
    Eigen::SparseMatrix<double> nearest_neighbour_matrix = m.nearest_neighbour_matrix();

    for (int i = 0; i < NClosest1.rows(); i++) {
        for (int j = 0; j < NClosest1.cols(); j++) {
            EXPECT_EQ(NClosest1(i, j), nearest_neighbour_matrix.coeff(i, j))
                << "Difference in quadrature node at:" << i;
        }
    }

    nearest_neighbour_matrix = m.nearest_neighbour_matrix_by_dof();

    for (int i = 0; i < nu.rows(); i++) {
        for (int j = 0; j < nu.cols(); j++) {
            EXPECT_EQ(nu(i, j), nearest_neighbour_matrix.coeff(i, j)) << "Difference at:" << i << "," << j;
        }
    }
}


TEST_F(MatlabTest, test_resistance_matrix_assembly) {
    double epsilon = 0.02;

    cnearest::Vector xTrac = read_csv(data_folder + "xTrac.dat", 162);
    cnearest::Collection<cnearest::Vector3> force_positions;
    for (int i = 0; i < 54; i++) {
        force_positions.push_back({xTrac(i), xTrac(i + 54), xTrac(i + 108)});
    }
    cnearest::EmpiricalDiscretisation force_discretisation(force_positions);

    cnearest::Vector xQuad = read_csv(data_folder + "xQuad.dat", 2592);
    cnearest::Collection<cnearest::Vector3> quad_positions;
    for (int i = 0; i < 864; i++) {
        quad_positions.push_back({xQuad(i), xQuad(i + 864), xQuad(i + 864 + 864)});
    }
    cnearest::EmpiricalDiscretisation quad_discretisation(quad_positions);

    cnearest::Matrix nu_A = read_csv(data_folder + "nu_A.dat", 162, 162);

    // Create a collection with one rigid body
    cnearest::Collection<cnearest::RigidBody> bodies{
        cnearest::RigidBody(force_discretisation, quad_discretisation)};

    // Assign global ids to the nodes (trivial with one rigid body)
    cnearest::GlobalIndexAssignment index_assignment(bodies);

    // Detect the nearest neighbours
    cnearest::NearestNeighbourDetectionAlgorithm algorithm = matlab_matching_detection(index_assignment);
    auto nearest_neighbour_results = algorithm.nearest_neighbours(bodies);

    // Construct the assembled matrix
    cnearest::MatrixAssemblerByNNMultiplication<true> m(index_assignment, nearest_neighbour_results, epsilon);

    cnearest::Matrix resistance_matrix = m.generate_matrix();

    for (int i = 0; i < nu_A.rows(); i++) {
        for (int j = 0; j < nu_A.cols(); j++) {
            EXPECT_NEAR(nu_A(i, j), resistance_matrix(i, j), 1.0e-6) << "Difference at:" << i << "," << j;
        }
    }
}


TEST_F(MatlabTest, test_resistance_matrix_assembly_by_accuulation) {
    double epsilon = 0.02;

    cnearest::Vector xTrac = read_csv(data_folder + "xTrac.dat", 162);
    cnearest::Collection<cnearest::Vector3> force_positions;
    for (int i = 0; i < 54; i++) {
        force_positions.push_back({xTrac(i), xTrac(i + 54), xTrac(i + 108)});
    }
    cnearest::EmpiricalDiscretisation force_discretisation(force_positions);

    cnearest::Vector xQuad = read_csv(data_folder + "xQuad.dat", 2592);
    cnearest::Collection<cnearest::Vector3> quad_positions;
    for (int i = 0; i < 864; i++) {
        quad_positions.push_back({xQuad(i), xQuad(i + 864), xQuad(i + 864 + 864)});
    }
    cnearest::EmpiricalDiscretisation quad_discretisation(quad_positions);

    cnearest::Matrix nu_A = read_csv(data_folder + "nu_A.dat", 162, 162);

    // Create a collection with one rigid body
    cnearest::Collection<cnearest::RigidBody> bodies{
        cnearest::RigidBody(force_discretisation, quad_discretisation)};

    // Assign global ids to the nodes (trivial with one rigid body)
    cnearest::GlobalIndexAssignment index_assignment(bodies);

    // Detect the nearest neighbours
    cnearest::NearestNeighbourDetectionAlgorithm algorithm = matlab_matching_detection(index_assignment);
    auto nearest_neighbour_results = algorithm.nearest_neighbours(bodies);

    // Construct the assembled matrix
    cnearest::MatrixAssemblerByBlockAccumulation<true> m(index_assignment, nearest_neighbour_results, epsilon);

    cnearest::Matrix resistance_matrix = m.generate_matrix();

    for (int i = 0; i < nu_A.rows(); i++) {
        for (int j = 0; j < nu_A.cols(); j++) {
            EXPECT_NEAR(nu_A(i, j), resistance_matrix(i, j), 1.0e-6) << "Difference at:" << i << "," << j;
        }
    }
}