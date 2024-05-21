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
