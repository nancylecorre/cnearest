#include "gtest/gtest.h"
#include "discretisations/single_point_discretisation.h"
#include "discretisations/straight_line_discretisation.h"
#include "discretisations/uniform_spherical_discretisation.h"
#include "discretisations/fibonacci_sphere_discretisation.h"
#include "discretisations/tapered_rod_discretisation.h"
#include "discretisations/empirical_discretisation.h"
#include "discretisations/conglomerate_discretisation.h"

TEST(TestSinglePointDiscretisation, TestSinglePointDiscretisation) {
    
    cnearest::SinglePointDiscretisation discretisation(1, 2, 3);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 1);
    ASSERT_EQ(positions[0](0), 1);
    ASSERT_EQ(positions[0](1), 2);
    ASSERT_EQ(positions[0](2), 3);

}


TEST(TestLineDiscretisation, TestLineDiscretisation) {
    cnearest::Vector3 start_position = {1, 5, 7};
    cnearest::Vector3 end_position = {2, 8, 3};
    cnearest::StraightLineDiscretisation discretisation(start_position, end_position);

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
    cnearest::StraightLineDiscretisation discretisation(start_position, end_position, 1);

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
    cnearest::StraightLineDiscretisation discretisation(start_position, end_position, 5);

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



TEST(TestFibonacciSphericalDiscretisation, TestFibonacciSphericalDiscretisation) {
    constexpr double radius = 1.1;
    cnearest::FibonacciSphericalDiscretisation discretisation(radius, 100);

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
    cnearest::FibonacciSphericalDiscretisation discretisation(radius, 1000);

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


TEST(TestTaperedRodDiscretisation, TestTaperedRodDiscretisation) {
    constexpr double radius = 1.1;
    cnearest::TaperedRodDiscretisation discretisation(radius, 3, 2);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 4);
}

TEST(TestEmpiricalDiscretisation, TestEmpiricalDiscretisation) {
    cnearest::EmpiricalDiscretisation discretisation(
        {
            {1,2,3},
            {4,5,6},
            {-1.0,0.0,2.0}
        });

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 3);
    ASSERT_EQ(positions[0](0), 1);
    ASSERT_EQ(positions[0](1), 2);
    ASSERT_EQ(positions[0](2), 3);

    ASSERT_EQ(positions[1](0), 4);
    ASSERT_EQ(positions[1](1), 5);
    ASSERT_EQ(positions[1](2), 6);

    ASSERT_EQ(positions[2](0), -1.0);
    ASSERT_EQ(positions[2](1), 0);
    ASSERT_EQ(positions[2](2), 2);
}


TEST(TestConglomerateDiscretisation, TestConglomerateDiscretisation) {

    cnearest::SinglePointDiscretisation d1(1, 2, 3);
    cnearest::SinglePointDiscretisation d2(4, 5, 6);
    cnearest::SinglePointDiscretisation d3(-1, 0, 2);

    cnearest::ConglomerateDiscretisation discretisation(
        {&d1,&d2,&d3});

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 3);
    ASSERT_EQ(positions[0](0), 1);
    ASSERT_EQ(positions[0](1), 2);
    ASSERT_EQ(positions[0](2), 3);

    ASSERT_EQ(positions[1](0), 4);
    ASSERT_EQ(positions[1](1), 5);
    ASSERT_EQ(positions[1](2), 6);

    ASSERT_EQ(positions[2](0), -1.0);
    ASSERT_EQ(positions[2](1), 0);
    ASSERT_EQ(positions[2](2), 2);
}


TEST(TestConglomerateDiscretisation, TestConglomerateDiscretisation2) {
    cnearest::SinglePointDiscretisation d1(1, 2, 3);
    cnearest::SinglePointDiscretisation d2(4, 5, 6);

    cnearest::ConglomerateDiscretisation discretisation(d1, d2);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 2);
    ASSERT_EQ(positions[0](0), 1);
    ASSERT_EQ(positions[0](1), 2);
    ASSERT_EQ(positions[0](2), 3);

    ASSERT_EQ(positions[1](0), 4);
    ASSERT_EQ(positions[1](1), 5);
    ASSERT_EQ(positions[1](2), 6);

}