#include "quadrature_node.h"
#include "gtest/gtest.h"

class QuadratureNodesTest : public ::testing::Test {
   protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(QuadratureNodesTest, test_creation) {
    cnearest::Vector3 position = {1, 2, 3};
    cnearest::QuadratureNode node(position);
}

TEST_F(QuadratureNodesTest, test_position) {
    cnearest::Vector3 position = {1, 2, 3};
    cnearest::QuadratureNode node(position);

    ASSERT_EQ(node.position()(0), position(0));
    ASSERT_EQ(node.position()(1), position(1));
    ASSERT_EQ(node.position()(2), position(2));
}

class QuadratureNodesParameterizedTests : public ::testing::TestWithParam<cnearest::Vector3> {
};

TEST_P(QuadratureNodesParameterizedTests, TestPositions) {
    cnearest::Vector3 position = GetParam();
    cnearest::QuadratureNode node(position);
    ASSERT_EQ(node.position()(0), position(0));
    ASSERT_EQ(node.position()(1), position(1));
    ASSERT_EQ(node.position()(2), position(2));
}

INSTANTIATE_TEST_SUITE_P(
    QuadratureNodesParameterizedTests_in_Z, QuadratureNodesParameterizedTests,
    ::testing::Values(cnearest::Vector3({0, 0, std::numeric_limits<double>::max()}),
                      cnearest::Vector3({0, 0, 1e8}), cnearest::Vector3({0, 0, 1000}),
                      cnearest::Vector3({0, 0, 1.0}),
                      cnearest::Vector3({0, 0, std::numeric_limits<double>::min()}),
                      cnearest::Vector3({0, 0, 0.0}),
                      cnearest::Vector3({0, 0, -std::numeric_limits<double>::min()}),
                      cnearest::Vector3({0, 0, -1.0}), cnearest::Vector3({0, 0, -1000}),
                      cnearest::Vector3({0, 0, -1e8}),
                      cnearest::Vector3({0, 0, -std::numeric_limits<double>::min()})));

INSTANTIATE_TEST_SUITE_P(
    QuadratureNodesParameterizedTests_in_Y, QuadratureNodesParameterizedTests,
    ::testing::Values(cnearest::Vector3({0, std::numeric_limits<double>::max(), 0}),
                      cnearest::Vector3({0, 1e8, 0}), 
                      cnearest::Vector3({0, 1000, 0}),
                      cnearest::Vector3({0, 1.0, 0}),
                      cnearest::Vector3({0, std::numeric_limits<double>::min(), 0}),
                      cnearest::Vector3({0, 0.0, 0}),
                      cnearest::Vector3({0, -std::numeric_limits<double>::min(), 0}),
                      cnearest::Vector3({0, -1.0, 0}),
                      cnearest::Vector3({0, -1000, 0}),
                      cnearest::Vector3({0, -1e8, 0}),
                      cnearest::Vector3({0, -std::numeric_limits<double>::min(), 0})));

INSTANTIATE_TEST_SUITE_P(
    QuadratureNodesParameterizedTests_in_XY, QuadratureNodesParameterizedTests,
    ::testing::Values(cnearest::Vector3({std::numeric_limits<double>::max(),
                                         std::numeric_limits<double>::max(), 0}),
                      cnearest::Vector3({1e8, 1e8, 0}), 
                      cnearest::Vector3({0, 1000, 0}),
                      cnearest::Vector3({1.0, 1.0, 0}),
                      cnearest::Vector3({std::numeric_limits<double>::min(),
                                         std::numeric_limits<double>::min(), 0}),
                      cnearest::Vector3({0, 0.0, 0}),
                      cnearest::Vector3({-std::numeric_limits<double>::min(),
                                         -std::numeric_limits<double>::min(), 0}),
                      cnearest::Vector3({-1.0, -1.0, 0}), 
                      cnearest::Vector3({0, -1000, 0}),
                      cnearest::Vector3({-1e8, -1e8, 0}),
                      cnearest::Vector3({-std::numeric_limits<double>::min(),
                                         -std::numeric_limits<double>::min(), 0})));