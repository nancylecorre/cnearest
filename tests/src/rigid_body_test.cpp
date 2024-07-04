#include "rigid_body.h"
#include "discretisations/single_point_discretisation.h"
#include "gtest/gtest.h"

class RigidBodyTest : public ::testing::Test {
   protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(RigidBodyTest, test_creation) {
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::RigidBody body(discretisation, discretisation);

    ASSERT_EQ(body.force_nodes().size(), 1);
    ASSERT_EQ(body.quadrature_nodes().size(), 1);

}


TEST_F(RigidBodyTest, test_creation_with_displacement) {
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::RigidBody body(discretisation, discretisation, {2,4,5});

    ASSERT_EQ(body.force_nodes().size(), 1);
    ASSERT_EQ(body.force_nodes().nodes()[0].position().x(), 3.0);
    ASSERT_EQ(body.force_nodes().nodes()[0].position().y(), 6.0);
    ASSERT_EQ(body.force_nodes().nodes()[0].position().z(), 8.0);
    ASSERT_EQ(body.quadrature_nodes().size(), 1);
    ASSERT_EQ(body.quadrature_nodes().nodes()[0].position().x(), 3.0);
    ASSERT_EQ(body.quadrature_nodes().nodes()[0].position().y(), 6.0);
    ASSERT_EQ(body.quadrature_nodes().nodes()[0].position().z(), 8.0);
}


TEST_F(RigidBodyTest, test_creation_with_rotation) {

    cnearest::Matrix3 rotation;
    rotation << 1, 0, 0, 
        0, 0, -1, 
        0, 1, 0;

    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::RigidBody body(discretisation, discretisation, {2, 4, 5}, rotation);

    ASSERT_EQ(body.force_nodes().size(), 1);
    ASSERT_EQ(body.force_nodes().nodes()[0].position().x(), 3.0);
    ASSERT_EQ(body.force_nodes().nodes()[0].position().y(), 1.0);
    ASSERT_EQ(body.force_nodes().nodes()[0].position().z(), 7.0);
    ASSERT_EQ(body.quadrature_nodes().size(), 1);
    ASSERT_EQ(body.quadrature_nodes().nodes()[0].position().x(), 3.0);
    ASSERT_EQ(body.quadrature_nodes().nodes()[0].position().y(), 1.0);
    ASSERT_EQ(body.quadrature_nodes().nodes()[0].position().z(), 7.0);
}


TEST_F(RigidBodyTest, test_origin_point) {
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    cnearest::RigidBody body(discretisation, discretisation, {2, 4, 5});

    ASSERT_EQ(body.origin_point().size(), 3);
    ASSERT_EQ(body.origin_point().x(), 2.0);
    ASSERT_EQ(body.origin_point().y(), 4.0);
    ASSERT_EQ(body.origin_point().z(), 5.0);
}