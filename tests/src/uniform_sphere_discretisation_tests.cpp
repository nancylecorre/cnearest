#include "discretisations/uniform_spherical_discretisation.h"
#include "gtest/gtest.h"



TEST(TestUniformSphereDiscretisation, test_cube_discretisation_2) {
    cnearest::UnitCubeDiscretisation discretisation(2);
    auto positions = discretisation.positions();
    ASSERT_EQ(positions.size(), 24);
    for (auto& position : positions) {
        ASSERT_EQ(position.cwiseAbs().maxCoeff(), 1.0)
            << "there should always be exactly one biggest dof with magnitude 1 or -1";
    }
}

TEST(TestUniformSphereDiscretisation, test_cube_discretisation_20) {
    cnearest::UnitCubeDiscretisation discretisation(20);
    auto positions = discretisation.positions();
    ASSERT_EQ(positions.size(), 6*20*20);
    for (auto& position : positions) {
        ASSERT_EQ(position.cwiseAbs().maxCoeff(), 1.0)
            << "there should always be exactly one biggest dof with magnitude 1 or -1";
    }
}

TEST(TestUniformSphericalDiscretisation, TestUniformSphericalDiscretisation) {
    constexpr double radius = 1.1;
    cnearest::UniformSphericalDiscretisation discretisation(radius, 1);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 6);

    // check all points are on the sphere
    for (auto& position : positions) {
        ASSERT_NEAR(position.norm(), radius, 1.0e-15);
    }

    // check there are no near-duplicate points
    for (auto& position : positions) {
        for (auto& position2 : positions) {
            if (position != position2) {
                EXPECT_GT((position - position2).norm(), radius / M_PI / 6)
                    << "Positions were too close together, position : " << position
                    << " vs " << position2;
            }
        }
    }
}

TEST(TestUniformSphericalDiscretisation,
     test_uniform_spherical_2) {
    constexpr double radius = 1.1;
    constexpr int msph = 2;
    cnearest::UniformSphericalDiscretisation discretisation(radius, msph);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), msph * msph * 6);

    // check all points are on the sphere
    for (auto& position : positions) {
        ASSERT_NEAR(position.norm(), radius, 1.0e-15);
    }

    // check there are no near-duplicate points
    for (auto& position : positions) {
        for (auto& position2 : positions) {
            if (position != position2) {
                EXPECT_GT((position - position2).norm(),
                          radius / M_PI / (msph * msph * 6))
                    << "Positions were too close together, position : " << position
                    << " vs " << position2;
            }
        }
    }
}

    TEST(TestUniformSphericalDiscretisation, test_uniform_spherical_6) {
    constexpr double radius = 1.1;
    constexpr int msph = 6;
    cnearest::UniformSphericalDiscretisation discretisation(radius, msph);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), msph * msph * 6);

    // check all points are on the sphere
    for (auto& position : positions) {
        ASSERT_NEAR(position.norm(), radius, 1.0e-15);
    }

    // check there are no near-duplicate points
    for (auto& position : positions) {
        for (auto& position2 : positions) {
            if (position != position2) {
                EXPECT_GT((position - position2).norm(),
                          radius / M_PI / (msph * msph * 6))
                    << "Positions were too close together, position : " << position
                    << " vs " << position2;
            }
        }
    }
}