#include "rigid_body.h"
#include "discretisations/single_point_discretisation.h"
#include "discretisations/uniform_spherical_discretisation.h"
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



TEST_F(RigidBodyTest, test_discretisation_spacing) {

    

    cnearest::UniformSphericalDiscretisation discretisation1(1.0, 1);
    cnearest::UniformSphericalDiscretisation discretisation2(1.0, 2);
    cnearest::UniformSphericalDiscretisation discretisation3(1.0, 3);
    cnearest::UniformSphericalDiscretisation discretisation6(1.0, 6);
    cnearest::UniformSphericalDiscretisation discretisation12(1.0, 12);
    cnearest::UniformSphericalDiscretisation discretisation24(1.0, 24);
    cnearest::UniformSphericalDiscretisation discretisation48(1.0, 48);
    cnearest::UniformSphericalDiscretisation discretisation96(1.0, 96);
    cnearest::UniformSphericalDiscretisation discretisation192(1.0, 192);

    cnearest::RigidBody body(discretisation1, discretisation2);

    ASSERT_EQ(body.force_nodes().size(), 6);
    //ASSERT_EQ(body.force_discretisation_spacing(), 1);
    ASSERT_EQ(body.quadrature_nodes().size(), 24);
    //ASSERT_EQ(body.quadrature_discretisation_spacing(), 1);

    
    body = cnearest::RigidBody(discretisation3, discretisation6);

    ASSERT_EQ(body.force_nodes().size(), 54);
    ASSERT_NEAR(body.force_discretisation_spacing(), 0.5796, 0.001);
    ASSERT_EQ(body.quadrature_nodes().size(), 216);
    ASSERT_NEAR(body.quadrature_discretisation_spacing(), 0.2942, 0.001);

    
    
    body = cnearest::RigidBody(discretisation12, discretisation24);

    ASSERT_EQ(body.force_nodes().size(), 864);
    ASSERT_NEAR(body.force_discretisation_spacing(), 0.1611, 0.001);
    ASSERT_EQ(body.quadrature_nodes().size(), 3456);
    ASSERT_NEAR(body.quadrature_discretisation_spacing(), 0.0826, 0.001);

 /* These take too long to run in general (exp debug) testing
 
    body = cnearest::RigidBody(discretisation48, discretisation96);

    ASSERT_EQ(body.force_nodes().size(), 13824);
    ASSERT_NEAR(body.force_discretisation_spacing(), 0.0416, 0.001);
    ASSERT_EQ(body.quadrature_nodes().size(), 55296);
    ASSERT_NEAR(body.quadrature_discretisation_spacing(), 0.0208, 0.001);

    
    body = cnearest::RigidBody(discretisation192, discretisation1);

    ASSERT_EQ(body.force_nodes().size(), 221184);
    ASSERT_NEAR(body.force_discretisation_spacing(), 0.0104, 0.001);
 */   
}