#include "gtest/gtest.h"
#include "discretisations/single_point_discretisation.h"
#include "discretisations/line_discretisation.h"
#include "discretisations/uniform_spherical_discretisation.h"
#include "discretisations/fibonacci_sphere_discretisation.h"

TEST(TestSinglePointDiscretisation, TestSinglePointDiscretisation) {
    
    cnearest::SinglePointDescretisation discretisation(1, 2, 3);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 1);
    ASSERT_EQ(positions[0](0), 1);
    ASSERT_EQ(positions[0](1), 2);
    ASSERT_EQ(positions[0](2), 3);

}


TEST(TestLineDiscretisation, TestLineDiscretisation) {
    cnearest::Vector3 start_position = {1, 5, 7};
    cnearest::Vector3 end_position = {2, 8, 3};
    cnearest::LineDescretisation discretisation(start_position, end_position);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 2);
    ASSERT_EQ(positions[0](0), 1);
    ASSERT_EQ(positions[0](1), 5);
    ASSERT_EQ(positions[0](2), 7);

    ASSERT_EQ(positions[1](0), 2);
    ASSERT_EQ(positions[1](1), 8);
    ASSERT_EQ(positions[1](2), 3);
}


TEST(TestLineDiscretisation, TestLineDiscretisationWithMidNode) {
    cnearest::Vector3 start_position = {1, 5, 7};
    cnearest::Vector3 end_position = {2, 8, 3};
    cnearest::LineDescretisation discretisation(start_position, end_position, 1);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 3);
    ASSERT_EQ(positions[0](0), 1);
    ASSERT_EQ(positions[0](1), 5);
    ASSERT_EQ(positions[0](2), 7);

    ASSERT_EQ(positions[1](0), 1.5);
    ASSERT_EQ(positions[1](1), 6.5);
    ASSERT_EQ(positions[1](2), 5);

    ASSERT_EQ(positions[2](0), 2);
    ASSERT_EQ(positions[2](1), 8);
    ASSERT_EQ(positions[2](2), 3);
}


TEST(TestLineDiscretisation, TestLineDiscretisationWith5MidNodes) {
    cnearest::Vector3 start_position = {1, 5, 7};
    cnearest::Vector3 end_position = {2, 8, 3};
    cnearest::LineDescretisation discretisation(start_position, end_position, 5);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 7);
    ASSERT_EQ(positions[0](0), 1);
    ASSERT_EQ(positions[0](1), 5);
    ASSERT_EQ(positions[0](2), 7);

    ASSERT_EQ(positions[3](0), 1.5);
    ASSERT_EQ(positions[3](1), 6.5);
    ASSERT_EQ(positions[3](2), 5);

    ASSERT_EQ(positions[6](0), 2);
    ASSERT_EQ(positions[6](1), 8);
    ASSERT_EQ(positions[6](2), 3);
}


TEST(TestUniformSphericalDiscretisation, TestUniformSphericalDiscretisation) {
    constexpr double radius = 1.1;
    cnearest::UniformSphericalDescretisation discretisation(radius, 1);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 6);

    //check all points are on the sphere
    for (auto& position : positions) {
        ASSERT_NEAR(position.norm(), radius, 1.0e-15);
    }

    //check there are no near-duplicate points
    for (auto& position : positions) {
        for (auto& position2 : positions) {
            if (position != position2) {
                EXPECT_GT((position - position2).norm(),
                          radius/M_PI / 6)
                    << "Positions were too close together, position : " << position << " vs "<<position2;
            
            }
        }
    }
}

TEST(TestUniformSphericalDiscretisation, TestUniformSphericalDiscretisationWithHigherNumber) {
    constexpr double radius = 1.1;
    constexpr int msph = 5;
    cnearest::UniformSphericalDescretisation discretisation(radius, msph);

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

TEST(TestFibonacciSphericalDiscretisation, TestFibonacciSphericalDiscretisation) {
    constexpr double radius = 1.1;
    cnearest::FibonacciSphericalDescretisation discretisation(radius, 100);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 100);

    // check all points are on the sphere
    for (auto& position : positions) {
        ASSERT_NEAR(position.norm(), radius, 1.0e-15);
    }

    // check there are no near-duplicate points
    for (auto& position : positions) {
        for (auto& position2 : positions) {
            if (position != position2) {
                EXPECT_GT((position - position2).norm(), radius / M_PI / 100)
                    << "Positions were too close together, position : " << position
                    << " vs " << position2;
            }
        }
    }
}


TEST(TestFibonacciSphericalDiscretisation, TestFibonacciSphericalDiscretisationHiger) {
    constexpr double radius = 1.1;
    cnearest::FibonacciSphericalDescretisation discretisation(radius, 1000);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 1000);

    // check all points are on the sphere
    for (auto& position : positions) {
        ASSERT_NEAR(position.norm(), radius, 1.0e-15);
    }

    // check there are no near-duplicate points
    for (auto& position : positions) {
        for (auto& position2 : positions) {
            if (position != position2) {
                EXPECT_GT((position - position2).norm(), radius / M_PI / 1000)
                    << "Positions were too close together, position : " << position
                    << " vs " << position2;
            }
        }
    }
}