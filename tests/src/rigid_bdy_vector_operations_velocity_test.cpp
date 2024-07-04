#include "gtest/gtest.h"
#include "discretisations/single_point_discretisation.h"
#include "rigid_body_vector_operations.h"



TEST(RigidBodyVectorOperationsNodalVelocitiesTests, test_creation) {
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::RigidBody body(discretisation, discretisation);

    cnearest::RigidBodyVectorOperations operations(body);

    ASSERT_EQ(&operations.rigid_body(), &body);

}


TEST(RigidBodyVectorOperationsNodalVelocitiesTests, test_linear_velocity) {

    cnearest::Vector3 rigid_body_centre = {0, 0, 0};
    cnearest::Vector3 node_offset = {0, 0, 0};
    cnearest::SixDOFVector rigid_body_velocity = {{0, 0, 1}, {0, 0, 0}};

    cnearest::Vector3 expected_nodal_velocity = {0, 0, 1};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    auto speeds = operations.determine_nodal_speeds(rigid_body_velocity);

    ASSERT_DOUBLE_EQ(speeds.at(sole_node).x(), expected_nodal_velocity.x());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).y(), expected_nodal_velocity.y());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).z(), expected_nodal_velocity.z());
}

TEST(RigidBodyVectorOperationsNodalVelocitiesTests, test_angular_velocity) {
    cnearest::Vector3 rigid_body_centre = {0, 0, 0};
    cnearest::Vector3 node_offset = {1, 0, 0};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    cnearest::SixDOFVector rigid_body_velocity = {{0, 0, 0}, {0, 0, 1}};
    cnearest::Vector3 expected_nodal_velocity = {0, 1, 0};
    auto speeds = operations.determine_nodal_speeds(rigid_body_velocity);
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).x(), expected_nodal_velocity.x());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).y(), expected_nodal_velocity.y());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).z(), expected_nodal_velocity.z());

}

TEST(RigidBodyVectorOperationsNodalVelocitiesTests, test_angular_velocities) {
    cnearest::Vector3 rigid_body_centre = {0, 0, 0};
    cnearest::Vector3 node_offset = {0, 0, 1};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    cnearest::SixDOFVector rigid_body_velocity = {{0, 0, 0}, {0, 0, 1}};
    cnearest::Vector3 expected_nodal_velocity = {0, 0, 0};
    auto speeds = operations.determine_nodal_speeds(rigid_body_velocity);
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).x(), expected_nodal_velocity.x());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).y(), expected_nodal_velocity.y());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).z(), expected_nodal_velocity.z());

    
    rigid_body_velocity.angular_component = {0, 1, 0};
    expected_nodal_velocity = {1, 0, 0};
    speeds = operations.determine_nodal_speeds(rigid_body_velocity);
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).x(), expected_nodal_velocity.x());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).y(), expected_nodal_velocity.y());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).z(), expected_nodal_velocity.z());

    rigid_body_velocity.angular_component = {1, 0, 0};
    expected_nodal_velocity = {0, -1, 0};
    speeds = operations.determine_nodal_speeds(rigid_body_velocity);
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).x(), expected_nodal_velocity.x());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).y(), expected_nodal_velocity.y());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).z(), expected_nodal_velocity.z());
}


class RigidBodyVectorOperationsNodalVelocitiesTests : public ::testing::TestWithParam<cnearest::Vector3> {};


TEST_P(RigidBodyVectorOperationsNodalVelocitiesTests, linear_speeds) { 
    
    cnearest::Vector3 data = GetParam();

    cnearest::Vector3 rigid_body_centre = cnearest::Vector3::Random();
    cnearest::Vector3 node_offset = cnearest::Vector3::Random();
    cnearest::SixDOFVector rigid_body_velocity = {data, {0, 0, 0}};

    cnearest::Vector3 expected_nodal_velocity = data;
    
    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), 
                                                       node_offset.y(),
                                                       node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    auto speeds = operations.determine_nodal_speeds(rigid_body_velocity);

    ASSERT_DOUBLE_EQ(speeds.at(sole_node).x(), expected_nodal_velocity.x());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).y(), expected_nodal_velocity.y());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).z(), expected_nodal_velocity.z());

}


TEST_P(RigidBodyVectorOperationsNodalVelocitiesTests, angular_velocity_with_no_offset) {
    cnearest::Vector3 data = GetParam();

    cnearest::Vector3 rigid_body_centre = cnearest::Vector3::Random();
    cnearest::Vector3 node_offset = {0, 0, 0};
    cnearest::SixDOFVector rigid_body_velocity = {{0, 0, 0}, data};

    cnearest::Vector3 expected_nodal_velocity = {0, 0, 0};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    auto speeds = operations.determine_nodal_speeds(rigid_body_velocity);

    ASSERT_DOUBLE_EQ(speeds.at(sole_node).x(), expected_nodal_velocity.x());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).y(), expected_nodal_velocity.y());
    ASSERT_DOUBLE_EQ(speeds.at(sole_node).z(), expected_nodal_velocity.z());
}

TEST_P(RigidBodyVectorOperationsNodalVelocitiesTests, angular_velocity_with_alligned_offset) {
    cnearest::Vector3 data = GetParam();

    cnearest::Vector3 rigid_body_centre = cnearest::Vector3::Random();
    cnearest::Vector3 node_offset = data;
    cnearest::SixDOFVector rigid_body_velocity = {{0, 0, 0}, data};

    cnearest::Vector3 expected_nodal_velocity = {0, 0, 0};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    auto speeds = operations.determine_nodal_speeds(rigid_body_velocity);

    ASSERT_NEAR(speeds.at(sole_node).x(), expected_nodal_velocity.x(), 1.0e-12);
    ASSERT_NEAR(speeds.at(sole_node).y(), expected_nodal_velocity.y(), 1.0e-12);
    ASSERT_NEAR(speeds.at(sole_node).z(), expected_nodal_velocity.z(), 1.0e-12);
}

TEST_P(RigidBodyVectorOperationsNodalVelocitiesTests, angular_velocity_with_z_offset) {
    cnearest::Vector3 data = GetParam();

    cnearest::Vector3 rigid_body_centre = cnearest::Vector3::Random();
    cnearest::Vector3 node_offset = {0, 0, 1};
    cnearest::SixDOFVector rigid_body_velocity = {{0, 0, 0}, data};

    cnearest::Vector3 expected_nodal_velocity = {data.y(), -data.x(), 0};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    auto speeds = operations.determine_nodal_speeds(rigid_body_velocity);

    ASSERT_NEAR(speeds.at(sole_node).x(), expected_nodal_velocity.x(), 1.0e-12);
    ASSERT_NEAR(speeds.at(sole_node).y(), expected_nodal_velocity.y(), 1.0e-12);
    ASSERT_NEAR(speeds.at(sole_node).z(), expected_nodal_velocity.z(), 1.0e-12);
}


TEST_P(RigidBodyVectorOperationsNodalVelocitiesTests, offset_with_z_angular_velocity) {
    cnearest::Vector3 data = GetParam();

    cnearest::Vector3 rigid_body_centre = cnearest::Vector3::Random();
    cnearest::Vector3 node_offset = data;
    cnearest::SixDOFVector rigid_body_velocity = {{0, 0, 0}, {0, 0, 1}};

    cnearest::Vector3 expected_nodal_velocity = {-data.y(), data.x(), 0};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    auto speeds = operations.determine_nodal_speeds(rigid_body_velocity);

    ASSERT_NEAR(speeds.at(sole_node).x(), expected_nodal_velocity.x(), 1.0e-12);
    ASSERT_NEAR(speeds.at(sole_node).y(), expected_nodal_velocity.y(), 1.0e-12);
    ASSERT_NEAR(speeds.at(sole_node).z(), expected_nodal_velocity.z(), 1.0e-12);
}

INSTANTIATE_TEST_SUITE_P(
    TestSinglePoint,RigidBodyVectorOperationsNodalVelocitiesTests,
                         ::testing::Values(cnearest::Vector3{0, 0, 0}, 
                             cnearest::Vector3{0, 0, 1}, cnearest::Vector3{0, 1, 0}, cnearest::Vector3{1, 0, 0}, 
                             cnearest::Vector3{0, 1, 1}, cnearest::Vector3{1, 0, 1}, cnearest::Vector3{1, 1, 0}, 
                             cnearest::Vector3{1, 1, 1}, 
                             cnearest::Vector3{0, 0, -1}, cnearest::Vector3{0, -1, 0}, cnearest::Vector3{-1, 0, 0}, 
                             cnearest::Vector3{0, -1, -1}, cnearest::Vector3{-1, 0, -1}, cnearest::Vector3{-1, -1, 0}, 
                             cnearest::Vector3{-1, -1, -1}, 
                             cnearest::Vector3{0.25, 0.26, 0.27}, 
                             cnearest::Vector3{1.74, -2.36, 0.2}, 
                             cnearest::Vector3{-2.6, 100.0, 55.36},
                             cnearest::Vector3{-5, 3.6, -3.9}, 
                             cnearest::Vector3{1, 3.68, -78}
                         ));
