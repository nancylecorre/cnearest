#include "gtest/gtest.h"
#include "discretisations/single_point_discretisation.h"
#include "discretisations/straight_line_discretisation.h"
#include "rigid_body_vector_operations.h"




TEST(RigidBodyVectorOperationsForceTests, test_linear_force) {

    cnearest::Vector3 rigid_body_centre = {0, 0, 0};
    cnearest::Vector3 node_offset = {0, 0, 0};
    cnearest::Vector3 nodal_force = {0, 0, 1};

    cnearest::SixDOFVector expected_rigid_body_force = {{0, 0, 1}, {0, 0, 0}};


    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    std::map<const cnearest::ForceNode*, cnearest::Vector3> forces = {{sole_node, nodal_force}};
    auto force = operations.sum_nodal_forces(forces);

    ASSERT_DOUBLE_EQ(force.linear_component.x(), expected_rigid_body_force.linear_component.x());
    ASSERT_DOUBLE_EQ(force.linear_component.y(), expected_rigid_body_force.linear_component.y());
    ASSERT_DOUBLE_EQ(force.linear_component.z(), expected_rigid_body_force.linear_component.z());

    
    ASSERT_DOUBLE_EQ(force.angular_component.x(), expected_rigid_body_force.angular_component.x());
    ASSERT_DOUBLE_EQ(force.angular_component.y(), expected_rigid_body_force.angular_component.y());
    ASSERT_DOUBLE_EQ(force.angular_component.z(), expected_rigid_body_force.angular_component.z());
}

TEST(RigidBodyVectorOperationsForceTests, test_angular_force) {
    cnearest::Vector3 rigid_body_centre = {0, 0, 0};
    cnearest::Vector3 node_offset = {0, 0, 1};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    std::map<const cnearest::ForceNode*, cnearest::Vector3> forces = {{sole_node, {0, 0, 1}}};
    cnearest::SixDOFVector expected_rigid_body_force = {{0, 0, 1}, {0, 0, 0}};
    auto force = operations.sum_nodal_forces(forces);
    ASSERT_DOUBLE_EQ(force.linear_component.x(), expected_rigid_body_force.linear_component.x());
    ASSERT_DOUBLE_EQ(force.linear_component.y(), expected_rigid_body_force.linear_component.y());
    ASSERT_DOUBLE_EQ(force.linear_component.z(), expected_rigid_body_force.linear_component.z());

    ASSERT_DOUBLE_EQ(force.angular_component.x(), expected_rigid_body_force.angular_component.x());
    ASSERT_DOUBLE_EQ(force.angular_component.y(), expected_rigid_body_force.angular_component.y());
    ASSERT_DOUBLE_EQ(force.angular_component.z(), expected_rigid_body_force.angular_component.z());

    
    forces = {{sole_node, {1, 0, 0}}};
    expected_rigid_body_force = {{1, 0, 0}, {0, 1, 0}};
    force = operations.sum_nodal_forces(forces);
    ASSERT_DOUBLE_EQ(force.linear_component.x(), expected_rigid_body_force.linear_component.x());
    ASSERT_DOUBLE_EQ(force.linear_component.y(), expected_rigid_body_force.linear_component.y());
    ASSERT_DOUBLE_EQ(force.linear_component.z(), expected_rigid_body_force.linear_component.z());

    ASSERT_DOUBLE_EQ(force.angular_component.x(), expected_rigid_body_force.angular_component.x());
    ASSERT_DOUBLE_EQ(force.angular_component.y(), expected_rigid_body_force.angular_component.y());
    ASSERT_DOUBLE_EQ(force.angular_component.z(), expected_rigid_body_force.angular_component.z());

    forces = {{sole_node, {0, 1, 0}}};
    expected_rigid_body_force = {{0, 1, 0}, {-1, 0, 0}};
    force = operations.sum_nodal_forces(forces);
    ASSERT_DOUBLE_EQ(force.linear_component.x(), expected_rigid_body_force.linear_component.x());
    ASSERT_DOUBLE_EQ(force.linear_component.y(), expected_rigid_body_force.linear_component.y());
    ASSERT_DOUBLE_EQ(force.linear_component.z(), expected_rigid_body_force.linear_component.z());

    ASSERT_DOUBLE_EQ(force.angular_component.x(), expected_rigid_body_force.angular_component.x());
    ASSERT_DOUBLE_EQ(force.angular_component.y(), expected_rigid_body_force.angular_component.y());
    ASSERT_DOUBLE_EQ(force.angular_component.z(), expected_rigid_body_force.angular_component.z());
}


TEST(RigidBodyVectorOperationsForceTests, test_summing_2_forces) {
    cnearest::Vector3 rigid_body_centre = {0, 0, 0};
    cnearest::Vector3 nodal_force = {0, 0, 1};

    cnearest::SixDOFVector expected_rigid_body_force = {{0, 0, 2}, {0, 0, 0}};

    cnearest::StraightLineDiscretisation discretisation({0, 0, -1}, {0, 0, 1});

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto node0 = &body.force_nodes().nodes()[0];
    auto node1 = &body.force_nodes().nodes()[1];

    cnearest::RigidBodyVectorOperations operations(body);

    std::map<const cnearest::ForceNode*, cnearest::Vector3> forces = {
        {node0, nodal_force}, {node1, nodal_force}};
    auto force = operations.sum_nodal_forces(forces);

    ASSERT_DOUBLE_EQ(force.linear_component.x(), expected_rigid_body_force.linear_component.x());
    ASSERT_DOUBLE_EQ(force.linear_component.y(), expected_rigid_body_force.linear_component.y());
    ASSERT_DOUBLE_EQ(force.linear_component.z(), expected_rigid_body_force.linear_component.z());

    ASSERT_DOUBLE_EQ(force.angular_component.x(), expected_rigid_body_force.angular_component.x());
    ASSERT_DOUBLE_EQ(force.angular_component.y(), expected_rigid_body_force.angular_component.y());
    ASSERT_DOUBLE_EQ(force.angular_component.z(), expected_rigid_body_force.angular_component.z());
}

TEST(RigidBodyVectorOperationsForceTests, test_summing_2_moments) {
    cnearest::Vector3 rigid_body_centre = {0, 0, 0};
    cnearest::Vector3 nodal_force = {0, 1, 0};

    cnearest::SixDOFVector expected_rigid_body_force = {{0, 2, 0}, {0, 0, 0.4}};

    cnearest::StraightLineDiscretisation discretisation({-1, 0, 0}, {1.4, 0, 0});

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto node0 = &body.force_nodes().nodes()[0];
    auto node1 = &body.force_nodes().nodes()[1];

    cnearest::RigidBodyVectorOperations operations(body);

    std::map<const cnearest::ForceNode*, cnearest::Vector3> forces = {{node0, nodal_force},
                                                                      {node1, nodal_force}};
    auto force = operations.sum_nodal_forces(forces);

    ASSERT_DOUBLE_EQ(force.linear_component.x(), expected_rigid_body_force.linear_component.x());
    ASSERT_DOUBLE_EQ(force.linear_component.y(), expected_rigid_body_force.linear_component.y());
    ASSERT_DOUBLE_EQ(force.linear_component.z(), expected_rigid_body_force.linear_component.z());

    ASSERT_DOUBLE_EQ(force.angular_component.x(), expected_rigid_body_force.angular_component.x());
    ASSERT_DOUBLE_EQ(force.angular_component.y(), expected_rigid_body_force.angular_component.y());
    ASSERT_DOUBLE_EQ(force.angular_component.z(), expected_rigid_body_force.angular_component.z());
}

class RigidBodyVectorOperationsForceTests : public ::testing::TestWithParam<cnearest::Vector3> {};




TEST_P(RigidBodyVectorOperationsForceTests, force_with_no_offset) {
    cnearest::Vector3 data = GetParam();

    cnearest::Vector3 rigid_body_centre = cnearest::Vector3::Random();
    cnearest::Vector3 node_offset = {0, 0, 0};
    cnearest::Vector3 nodal_force = data;

    cnearest::SixDOFVector expected_rigid_body_force = {data, {0, 0, 0}};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    std::map<const cnearest::ForceNode*, cnearest::Vector3> forces = {{sole_node, nodal_force}};
    auto force = operations.sum_nodal_forces(forces);

    ASSERT_DOUBLE_EQ(force.linear_component.x(), expected_rigid_body_force.linear_component.x());
    ASSERT_DOUBLE_EQ(force.linear_component.y(), expected_rigid_body_force.linear_component.y());
    ASSERT_DOUBLE_EQ(force.linear_component.z(), expected_rigid_body_force.linear_component.z());

    ASSERT_DOUBLE_EQ(force.angular_component.x(), expected_rigid_body_force.angular_component.x());
    ASSERT_DOUBLE_EQ(force.angular_component.y(), expected_rigid_body_force.angular_component.y());
    ASSERT_DOUBLE_EQ(force.angular_component.z(), expected_rigid_body_force.angular_component.z());
}

TEST_P(RigidBodyVectorOperationsForceTests, force_with_alligned_offset) {
    cnearest::Vector3 data = GetParam();

    cnearest::Vector3 rigid_body_centre = cnearest::Vector3::Random();
    cnearest::Vector3 node_offset = data;
    cnearest::Vector3 nodal_force = data;

    cnearest::SixDOFVector expected_rigid_body_force = {data, {0, 0, 0}};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    std::map<const cnearest::ForceNode*, cnearest::Vector3> forces = {{sole_node, nodal_force}};
    auto force = operations.sum_nodal_forces(forces);

    ASSERT_NEAR(force.linear_component.x(), expected_rigid_body_force.linear_component.x(), 1.0e-14);
    ASSERT_NEAR(force.linear_component.y(), expected_rigid_body_force.linear_component.y(), 1.0e-14);
    ASSERT_NEAR(force.linear_component.z(), expected_rigid_body_force.linear_component.z(), 1.0e-14);

    ASSERT_NEAR(force.angular_component.x(), expected_rigid_body_force.angular_component.x(), 1.0e-14);
    ASSERT_NEAR(force.angular_component.y(), expected_rigid_body_force.angular_component.y(), 1.0e-14);
    ASSERT_NEAR(force.angular_component.z(), expected_rigid_body_force.angular_component.z(), 1.0e-14);
}

TEST_P(RigidBodyVectorOperationsForceTests, force_with_z_offset) {
    cnearest::Vector3 data = GetParam();

    cnearest::Vector3 rigid_body_centre = cnearest::Vector3::Random();
    cnearest::Vector3 node_offset = {0, 0, 1};
    cnearest::Vector3 nodal_force = data;

    cnearest::SixDOFVector expected_rigid_body_force = {data, {-data.y(), data.x(), 0}};


    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    std::map<const cnearest::ForceNode*, cnearest::Vector3> forces = {{sole_node, nodal_force}};
    auto force = operations.sum_nodal_forces(forces);

    ASSERT_DOUBLE_EQ(force.linear_component.x(), expected_rigid_body_force.linear_component.x());
    ASSERT_DOUBLE_EQ(force.linear_component.y(), expected_rigid_body_force.linear_component.y());
    ASSERT_DOUBLE_EQ(force.linear_component.z(), expected_rigid_body_force.linear_component.z());

    ASSERT_DOUBLE_EQ(force.angular_component.x(), expected_rigid_body_force.angular_component.x());
    ASSERT_DOUBLE_EQ(force.angular_component.y(), expected_rigid_body_force.angular_component.y());
    ASSERT_DOUBLE_EQ(force.angular_component.z(), expected_rigid_body_force.angular_component.z());
}


TEST_P(RigidBodyVectorOperationsForceTests, offset_with_z_force) {
    cnearest::Vector3 data = GetParam();

    cnearest::Vector3 rigid_body_centre = cnearest::Vector3::Random();
    cnearest::Vector3 node_offset = data;
    cnearest::Vector3 nodal_force = {0, 0, 1};

    cnearest::SixDOFVector expected_rigid_body_force = {nodal_force, {data.y(), -data.x(), 0}};

    cnearest::SinglePointDiscretisation discretisation(node_offset.x(), node_offset.y(), node_offset.z());

    cnearest::RigidBody body(discretisation, discretisation, rigid_body_centre);
    auto sole_node = &body.force_nodes().nodes()[0];

    cnearest::RigidBodyVectorOperations operations(body);

    std::map<const cnearest::ForceNode*, cnearest::Vector3> forces = {{sole_node, nodal_force}};
    auto force = operations.sum_nodal_forces(forces);

    ASSERT_DOUBLE_EQ(force.linear_component.x(), expected_rigid_body_force.linear_component.x());
    ASSERT_DOUBLE_EQ(force.linear_component.y(), expected_rigid_body_force.linear_component.y());
    ASSERT_DOUBLE_EQ(force.linear_component.z(), expected_rigid_body_force.linear_component.z());

    ASSERT_DOUBLE_EQ(force.angular_component.x(), expected_rigid_body_force.angular_component.x());
    ASSERT_DOUBLE_EQ(force.angular_component.y(), expected_rigid_body_force.angular_component.y());
    ASSERT_DOUBLE_EQ(force.angular_component.z(), expected_rigid_body_force.angular_component.z());
}

INSTANTIATE_TEST_SUITE_P(
    TestSinglePoint, RigidBodyVectorOperationsForceTests,
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


